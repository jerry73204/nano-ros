use super::*;

/// An executor over `session` with a HOST clock installed.
///
/// phase-359 W10 — the core has no `std::time` fallback any more: the platform
/// API is the clock, so a build with no port (which the mock-session
/// configuration is, by construction — `mock` is `cfg(not(rmw-cffi))`) has none
/// and says so. A test that measures elapsed time therefore brings its own,
/// which is the same seam a board uses (`ExecutorConfig::clock_us`) and the
/// reason `from_session_with` exists (issue 0709).
///
/// `std::time::Instant` here is not a flavour leak: this file is test code, the
/// census excludes it, and a hosted test harness reading the host clock is
/// exactly right. What was wrong was the CORE offering the same thing as a
/// silent default.
/// Issue 1104 — serialize the tests that touch `time_source`'s process-global
/// and put back what was there.
///
/// `SIM_TIME_ACTIVE` is one flag for the whole process while an `Executor` is
/// one per test, so two tests that both care about it cannot run concurrently
/// and cannot leave it changed. This gives them both properties: the mutex
/// orders them, and the drop restores the value OBSERVED ON ENTRY rather than a
/// guessed default -- the previous cleanup wrote a literal `true`, which is only
/// right by coincidence and says nothing if the default ever moves.
///
/// The lock is only half the fix. It orders the tests that MEAN to touch the
/// flag; what stopped every other test from touching it by accident is
/// `Executor::sim_time_declared`, because `reconcile_ros_time_source` runs on
/// every `spin_once` of every executor.
#[cfg(feature = "sim-time")]
struct SimTimeGuard {
    was_active: bool,
    _lock: std::sync::MutexGuard<'static, ()>,
}

#[cfg(feature = "sim-time")]
impl SimTimeGuard {
    fn acquire() -> Self {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        // A poisoned lock means a sibling test panicked while holding it. That
        // is a failure being reported elsewhere, not a reason to fail here too,
        // so take the guard anyway and let the real failure be the one read.
        let _lock = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        Self {
            was_active: crate::time_source::is_active(),
            _lock,
        }
    }
}

#[cfg(feature = "sim-time")]
impl Drop for SimTimeGuard {
    fn drop(&mut self) {
        crate::time_source::set_active(self.was_active);
        nros_core::clock::Clock::clear_ros_time_override();
    }
}

fn test_clock_us() -> u64 {
    use std::{sync::OnceLock, time::Instant};
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    EPOCH.get_or_init(Instant::now).elapsed().as_micros() as u64
}

fn executor_with_clock(session: MockSession) -> Executor<'static> {
    let cfg = ExecutorConfig::default().clock_us(test_clock_us);
    Executor::from_session_with(session, &cfg)
}

use nros_core::{
    CdrReader, CdrWriter, DeserError, Deserialize, DeserializeView, LeSliceView, RosAction,
    RosMessage, SerError, Serialize, ViewableMessage,
};
use nros_rmw::{QoSProfile, TransportError};

use crate::{
    mock::{MockServiceServer, MockSession, MockSubscriber},
    timer::TimerDuration,
};

/// Sleep `ms` then call `spin_once(0)`. Phase 100 follow-up: spin_once
/// credits the wall-clock since the previous `spin_once` exited (not the
/// requested timeout) to the timer accumulator. Tests that previously
/// relied on `spin_once(N ms)` advancing virtual time by N must now
/// elapse real wall-clock time between calls. MockSession's `drive_io`
/// is a no-op, so the requested timeout adds no real elapsed.
// phase-359 W10 — sleeps through the platform, like the code under test does.
// This helper was the reason six tests below carried a `std` gate: it called
// `std::thread::sleep`, so everything that elapsed real time inherited the
// flavour. It now uses the same `nros_platform_sleep_us` the spin loops pace
// on, which also means these tests exercise that path rather than a parallel
// one.
#[cfg(feature = "alloc")]
fn elapse_then_spin_once(executor: &mut Executor, ms: u64) -> super::types::SpinOnceResult {
    super::spin::platform_sleep(core::time::Duration::from_millis(ms));
    executor.spin_once(core::time::Duration::from_millis(0))
}

/// Issue 0355 — a live-but-idle session (`MockSession::drive_io` returns
/// `Ok(())` with no data) must NEVER accumulate `session_io_failures`, no matter
/// how many idle spins elapse. The C spin loops
/// (`nros_executor_spin`/`nros_executor_spin_period`) gate their dead-session
/// bail on this counter; the bug was that they instead counted `spin_some`'s
/// idle `NROS_RET_TIMEOUT`, so a healthy C listener that idled longer than
/// `SPIN_ERROR_TOLERANCE * period` before its publisher was discovered got
/// killed — the CycloneDDS ros2→nano 0-delivery defect. Only a genuine
/// `drive_io` error may raise the counter.
#[test]
fn idle_spins_never_raise_session_io_failures() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    // Far more idle spins than the C-side SPIN_ERROR_TOLERANCE (16).
    for _ in 0..64 {
        let r = executor.spin_once(core::time::Duration::from_millis(0));
        assert!(!r.any_work(), "an idle mock session must do no work");
    }
    assert_eq!(
        executor.session_io_failures(),
        0,
        "idle spins over a live session must not accumulate io failures (issue 0355)"
    );
}

#[test]
fn test_error_conversion() {
    let transport_err = TransportError::ConnectionFailed;
    let node_err: NodeError = transport_err.into();
    assert_eq!(
        node_err,
        NodeError::Transport(TransportError::ConnectionFailed)
    );
}

// ====================================================================
// Mock types for arena callback tests
// ====================================================================

/// Simple test message: a single i32.
#[derive(Debug, Clone, PartialEq)]
struct TestMsg {
    data: i32,
}

impl RosMessage for TestMsg {
    const TYPE_NAME: &'static str = "test/msg/TestMsg";
    const TYPE_HASH: &'static str = "test_hash";
}

// Phase 212.K.7.6.b — minimal `Message` impl so the typed creators
// (which gain a `MessageForRmw` bound that tightens to
// `RosMessage + Message` under `rmw-cyclonedds`) still accept this
// test fixture. The codegen template emits both impls for real msg
// crates; here we mirror it by hand.
// phase-380 W4 — was `#[cfg(rmw_needs_type_descriptors)]`: the schema is
// now required by `MessageForRmw` on EVERY backend, because that is where
// a subscription's build-time size bound comes from.
impl nros_serdes::schema::Message for TestMsg {
    const TYPE_NAME: &'static str = "test/msg/TestMsg";
    const FIELDS: &'static [nros_serdes::schema::Field] = &[nros_serdes::schema::Field {
        name: "data",
        ty: nros_serdes::schema::FieldType::Int32,
        offset: 0,
    }];
}

impl Serialize for TestMsg {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.data)
    }
}

impl Deserialize for TestMsg {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            data: reader.read_i32()?,
        })
    }
}

/// CDR-encode a TestMsg(value) including CDR header.
fn encode_test_msg(value: i32) -> ([u8; 256], usize) {
    let mut buf = [0u8; 256];
    let mut writer = CdrWriter::new_with_header(&mut buf).unwrap();
    writer.write_i32(value).unwrap();
    let len = writer.position();
    (buf, len)
}

// ====================================================================
// Phase 403 W2 -- the type's bound sizes the receive buffer
//
// These MEASURE, they do not merely compile: `arena_used()` is the exact
// claimed total, so the delta across one registration IS the subscription's
// receive-buffer footprint. The phase's own rule is that no wave is done
// until a before/after exists on something that RUNS the changed code, and
// the buffered registration path is what runs it -- a backend advertising
// `supports_process_in_place` (zenoh) returns before allocating any buffer
// at all and is therefore NOT a site where this saves anything.
// ====================================================================

/// A message with no bound at any size, for the fallback arm. `TestMsg` is
/// bounded (one `Int32`), so it cannot exercise it.
#[derive(Debug, Clone, PartialEq)]
struct UnboundedTestMsg {
    data: heapless::String<8>,
}

impl RosMessage for UnboundedTestMsg {
    const TYPE_NAME: &'static str = "test/msg/UnboundedTestMsg";
    const TYPE_HASH: &'static str = "test_hash_unbounded";
}

impl nros_serdes::schema::Message for UnboundedTestMsg {
    const TYPE_NAME: &'static str = "test/msg/UnboundedTestMsg";
    const FIELDS: &'static [nros_serdes::schema::Field] = &[nros_serdes::schema::Field {
        name: "data",
        ty: nros_serdes::schema::FieldType::String,
        offset: 0,
    }];
}

impl Serialize for UnboundedTestMsg {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_string(self.data.as_str())
    }
}

impl Deserialize for UnboundedTestMsg {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        let s = reader.read_string()?;
        let mut data = heapless::String::new();
        // Truncation is fine: nothing here inspects the payload.
        for ch in s.chars() {
            if data.push(ch).is_err() {
                break;
            }
        }
        Ok(Self { data })
    }
}

/// Arena bytes one `TestMsg` subscription registration claims.
fn arena_delta(
    executor: &mut Executor<'_>,
    nid: super::node_record::NodeId,
    topic: &str,
    qos: QoSProfile,
    from_type: bool,
) -> usize {
    let before = executor.arena_used();
    if from_type {
        executor
            .node_mut(nid)
            .subscription(topic)
            .typed::<TestMsg>()
            .qos(qos)
            .rx_buffer_from_type()
            .build(|_: &TestMsg| {})
            .unwrap();
    } else {
        executor
            .node_mut(nid)
            .subscription(topic)
            .typed::<TestMsg>()
            .qos(qos)
            .build(|_: &TestMsg| {})
            .unwrap();
    }
    executor.arena_used() - before
}

fn qos_with_depth(depth: u32) -> QoSProfile {
    QoSProfile {
        depth,
        ..QoSProfile::default()
    }
}

/// The compatibility gate, the sibling of issue 0900's
/// `the_default_derivation_is_unchanged`: the knob exists so an image CAN
/// shrink its receive buffers, not so every image silently does.
///
/// Two depths rather than one absolute number, because the entry STRUCT's size
/// is a target detail this test has no business asserting while the SLOT size is
/// exactly what it must pin. `region_size` is linear in the slot size with a
/// depth-dependent slot count, so the difference between two depths isolates the
/// slot size: it comes out right only if a non-opted-in subscription is still
/// charged `DEFAULT_RX_BUF_SIZE` per slot. Sizing it from `TestMsg` (12 bytes)
/// instead fails here by 8128 bytes.
#[test]
fn the_default_subscription_buffer_is_unchanged() {
    let mut executor: Executor = executor_with_clock(MockSession::new());
    let nid = executor.node_builder("default_sizing").build().unwrap();

    let shallow = arena_delta(&mut executor, nid, "/shallow", qos_with_depth(2), false);
    let deep = arena_delta(&mut executor, nid, "/deep", qos_with_depth(10), false);

    let default = crate::config::DEFAULT_RX_BUF_SIZE;
    let (_s, shallow_region) = super::arena::buffered_region_size(2, default);
    let (_d, deep_region) = super::arena::buffered_region_size(10, default);

    assert_eq!(
        deep - shallow,
        deep_region - shallow_region,
        "a subscription that does not opt in must still be charged {default} \
         bytes per slot; depth 2 claimed {shallow} and depth 10 claimed {deep}"
    );
}

/// The saving, measured. A bounded type's slots shrink to its own bound.
///
/// Measured on this executor at the shipped defaults (`DEFAULT_RX_BUF_SIZE`
/// 1024, `QoSProfile::default().depth` 10, so an 11-slot `SpscRing`):
/// `TestMsg`'s bound is 12 bytes, the buffer region falls 11352 -> 220, and the
/// registration's total arena claim falls **13632 -> 2500 bytes**, i.e. 11132
/// recovered -- 15% of the 74240-byte default arena, from one subscription.
#[test]
fn a_bounded_type_shrinks_its_own_receive_buffer() {
    let bound = nros_serdes::size::max_serialized_bound::<TestMsg>()
        .expect("a single Int32 is a bounded type");
    assert!(
        bound < crate::config::DEFAULT_RX_BUF_SIZE,
        "this fixture is only meaningful while the bound ({bound}) is under the \
         default ({}) -- otherwise the assertion below passes for the wrong reason",
        crate::config::DEFAULT_RX_BUF_SIZE
    );

    let mut executor: Executor = executor_with_clock(MockSession::new());
    let nid = executor.node_builder("bound_sizing").build().unwrap();

    let depth = QoSProfile::default().depth;
    let default_delta = arena_delta(&mut executor, nid, "/plain", qos_with_depth(depth), false);
    let bound_delta = arena_delta(&mut executor, nid, "/sized", qos_with_depth(depth), true);

    let (_s, default_region) =
        super::arena::buffered_region_size(depth, crate::config::DEFAULT_RX_BUF_SIZE);
    let (_s2, bound_region) = super::arena::buffered_region_size(depth, bound);

    assert_eq!(
        default_delta - bound_delta,
        default_region - bound_region,
        "opting in must recover exactly the region difference: \
         default {default_delta} bytes vs bound-sized {bound_delta}"
    );
    assert!(
        bound_delta < default_delta,
        "the whole point is that it is smaller: {bound_delta} vs {default_delta}"
    );
}

/// An unbounded type yields NO number at all -- not the configured default.
///
/// Every message type is required to carry a bound (`.msg` `string<=64`, or a
/// `cap` in `nros-codegen.toml`), so `.rx_buffer_from_type()` on a type without
/// one is a BUILD error, and this is the predicate that error keys on. Phase
/// 380's rule is unchanged and is what makes erroring correct: `None` means "no
/// bound EXISTS", never "unknown", and inventing a number from a fallback is
/// what it forbids. There is no compile-fail harness in this workspace, so the
/// refusal is proved at the value the `const assert!` reads rather than by
/// compiling the rejected program.
#[test]
fn an_unbounded_type_yields_no_receive_buffer_size() {
    assert_eq!(
        nros_serdes::size::max_serialized_bound::<UnboundedTestMsg>(),
        None,
        "the fixture must actually be unbounded"
    );
    assert_eq!(
        crate::rmw_type_registry::subscription_rx_bytes::<UnboundedTestMsg>(
            crate::config::DEFAULT_RX_BUF_SIZE
        ),
        None,
        "an unbounded type must yield no size; falling back to the configured \
         default is the substitution phase 380 forbids"
    );

    // The non-opted-in path is untouched: an unbounded type still registers at
    // the configured default, because that path never claimed to size from the
    // type. Making an unbounded type impossible in the first place is a codegen
    // change, not this wave's.
    let mut executor: Executor = executor_with_clock(MockSession::new());
    let nid = executor.node_builder("unbounded_sizing").build().unwrap();
    let before = executor.arena_used();
    executor
        .node_mut(nid)
        .subscription("/unbounded_plain")
        .typed::<UnboundedTestMsg>()
        .build(|_: &UnboundedTestMsg| {})
        .unwrap();
    let (_slots, region) = super::arena::buffered_region_size(
        QoSProfile::default().depth,
        crate::config::DEFAULT_RX_BUF_SIZE,
    );
    assert!(
        executor.arena_used() - before > region,
        "an unbounded type that does not opt in still claims the default region"
    );
}

/// A message whose bound is NOT a multiple of 4 — one `int8`, so 4 bytes of
/// encapsulation plus 1.
///
/// `TestMsg` cannot test the framing allowance: a single `i32` bounds at 8,
/// which is already aligned, so rounding is a no-op and the assertion passes
/// whether or not the code rounds. This fixture exists so the test can fail.
/// Only `schema::Message` — `subscription_rx_bytes` asks for nothing else, and
/// the bound is a property of the schema rather than of the transport traits.
struct OddBoundTestMsg;

impl nros_serdes::schema::Message for OddBoundTestMsg {
    const TYPE_NAME: &'static str = "test/msg/OddBoundTestMsg";
    const FIELDS: &'static [nros_serdes::schema::Field] = &[nros_serdes::schema::Field {
        name: "data",
        ty: nros_serdes::schema::FieldType::Int8,
        offset: 0,
    }];
}

/// The receive buffer carries the transport's framing allowance, so it is the
/// bound rounded UP to 4 rather than the bound itself.
///
/// Measured, not assumed: a 25-byte message published by ROS 2 Humble over stock
/// `rmw_cyclonedds` reaches the Cyclone backend as 28 bytes — the RTPS
/// submessage's own alignment, applied by the sender. A buffer sized to the
/// exact bound refuses it, correctly and with the message lost. See
/// `rmw_type_registry::transport_framed`.
#[test]
fn the_receive_buffer_allows_for_transport_framing() {
    let bound = nros_serdes::size::max_serialized_bound::<OddBoundTestMsg>().unwrap();
    assert_ne!(
        bound % 4,
        0,
        "this fixture only proves anything while its bound is misaligned"
    );
    assert_eq!(
        crate::rmw_type_registry::subscription_rx_bytes::<OddBoundTestMsg>(
            crate::config::DEFAULT_RX_BUF_SIZE
        ),
        Some(bound.next_multiple_of(4)),
        "the receive buffer must hold what the transport delivers, not just what \
         the message measures"
    );

    // An already-aligned type must not be inflated.
    let aligned = nros_serdes::size::max_serialized_bound::<TestMsg>().unwrap();
    assert_eq!(aligned % 4, 0, "TestMsg is the aligned control case");
    assert_eq!(
        crate::rmw_type_registry::subscription_rx_bytes::<TestMsg>(
            crate::config::DEFAULT_RX_BUF_SIZE
        ),
        Some(aligned),
    );
}

/// A type whose bound EXCEEDS the ceiling keeps the ceiling. Growing here would
/// spend arena the caller never budgeted.
#[test]
fn a_bound_larger_than_the_ceiling_never_grows_the_buffer() {
    let bound = nros_serdes::size::max_serialized_bound::<TestMsg>().unwrap();
    assert_eq!(
        crate::rmw_type_registry::subscription_rx_bytes::<TestMsg>(bound - 1),
        Some(bound - 1),
        "the caller's ceiling wins; the bound may only shrink the buffer"
    );
}

// ====================================================================
// phase-408 W5b -- the hint sizes the ARENA on the info / validated C paths
//
// W5a routed `rx_buffer_hint` to the BACKEND (payload size class) and stopped
// there, because both entries still stored a real `[u8; RX_BUF]` and `RX_BUF`
// is `DEFAULT_RX_BUF_SIZE` at every call site. W5b moved that slot into the
// arena's trailing region, so the hint now buys the allocation too.
//
// These MEASURE. Each helper registers into a FRESH executor, so `arena_used()`
// starts from the same value every time and the delta differs ONLY by the
// trailing payload slot -- no alignment bookkeeping, no dependence on the entry
// struct's size, which is a target detail these tests have no business pinning.
// Before W5b every number below was identical.
// ====================================================================

unsafe extern "C" fn sizing_info_cb(
    _data: *const u8,
    _len: usize,
    _att: *const u8,
    _att_len: usize,
    _ctx: *mut core::ffi::c_void,
) {
}

/// Arena bytes ONE info-callback C subscription claims at `hint`.
fn info_arena_delta(hint: usize) -> usize {
    let mut executor: Executor = executor_with_clock(MockSession::new());
    let before = executor.arena_used();
    executor
        .add_arena_subscription_c_info_callback::<{ crate::config::DEFAULT_RX_BUF_SIZE }>(
            None,
            "/hinted",
            "test/msg/TestMsg",
            "test_hash",
            QoSProfile::default().keep_last(1),
            sizing_info_cb,
            core::ptr::null_mut(),
            hint,
        )
        .unwrap();
    executor.arena_used() - before
}

/// The saving, measured: the slot tracks the hint byte for byte.
///
/// Two hints rather than one absolute number, for the same reason
/// `the_default_subscription_buffer_is_unchanged` uses two depths -- the
/// difference isolates the payload slot from everything else the registration
/// claims.
#[test]
fn an_info_subscription_sizes_its_arena_slot_from_the_hint() {
    let small = info_arena_delta(128);
    let large = info_arena_delta(512);

    assert_eq!(
        large - small,
        512 - 128,
        "the trailing payload slot must track the hint: 128 claimed {small} \
         bytes, 512 claimed {large}"
    );
    // The delta is the payload slot PLUS the entry struct (an `RmwSubscriber`
    // and a 256-byte attachment staging area), so the absolute number is not a
    // thing to pin -- but it must move, and downwards, which is the whole
    // point: before W5b a hint changed nothing at all here.
    assert!(
        small < info_arena_delta(0),
        "a 128-byte hint must claim less than an unhinted registration; got \
         {small}"
    );
}

/// 0 is "this caller stated nothing", never "zero bytes". It falls back to the
/// configured default, so an unhinted registration claims what it always did.
#[test]
fn an_unhinted_info_subscription_still_claims_the_default() {
    let default = crate::config::DEFAULT_RX_BUF_SIZE;
    let unhinted = info_arena_delta(0);
    let hinted = info_arena_delta(128);

    assert_eq!(
        unhinted - hinted,
        default - 128,
        "a hint of 0 must fall back to {default} bytes, not to zero: unhinted \
         claimed {unhinted}, a 128-byte hint claimed {hinted}"
    );
    assert_eq!(
        unhinted,
        info_arena_delta(default),
        "stating the default explicitly and stating nothing must cost the same"
    );
}

#[cfg(feature = "safety-e2e")]
unsafe extern "C" fn sizing_validated_cb(
    _data: *const u8,
    _len: usize,
    _gap: i64,
    _duplicate: bool,
    _crc_valid: i8,
    _ctx: *mut core::ffi::c_void,
) {
}

/// The validated sibling carries the same change, and it is a SEPARATE
/// registration path -- fixing only the one the report named is how this
/// codebase's recurring defects recur.
#[cfg(feature = "safety-e2e")]
#[test]
fn a_validated_subscription_sizes_its_arena_slot_from_the_hint() {
    fn delta(hint: usize) -> usize {
        let mut executor: Executor = executor_with_clock(MockSession::new());
        let before = executor.arena_used();
        executor
            .add_arena_subscription_c_validated_callback::<{ crate::config::DEFAULT_RX_BUF_SIZE }>(
                None,
                "/hinted_validated",
                "test/msg/TestMsg",
                "test_hash",
                QoSProfile::default().keep_last(1),
                sizing_validated_cb,
                core::ptr::null_mut(),
                hint,
            )
            .unwrap();
        executor.arena_used() - before
    }

    let default = crate::config::DEFAULT_RX_BUF_SIZE;
    let small = delta(128);
    let large = delta(512);
    let unhinted = delta(0);

    assert_eq!(
        large - small,
        512 - 128,
        "the trailing payload slot must track the hint: 128 claimed {small} \
         bytes, 512 claimed {large}"
    );
    assert_eq!(
        unhinted - small,
        default - 128,
        "a hint of 0 must fall back to {default} bytes, not to zero: unhinted \
         claimed {unhinted}, a 128-byte hint claimed {small}"
    );
}

// ====================================================================
// Arena callback tests
// ====================================================================

#[test]
fn test_add_subscription_and_spin_once_no_data() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    let nid = executor
        .node_builder("test_add_subscription_and_spin_once_no_data")
        .build()
        .unwrap();

    // Register a subscription — callback should never fire
    let called = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let called2 = called.clone();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", move |_msg: &TestMsg| {
            called2.store(true, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 0);
    assert!(!result.any_work());
    assert!(!called.load(std::sync::atomic::Ordering::SeqCst));
}

#[test]
fn test_add_subscription_and_spin_once_with_data() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_add_subscription_and_spin_once_with_data")
        .build()
        .unwrap();
    let received = std::sync::Arc::new(std::sync::Mutex::new(None));
    let received2 = received.clone();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", move |msg: &TestMsg| {
            *received2.lock().unwrap() = Some(msg.data);
        })
        .unwrap();

    // Grab a pointer to the subscriber in the arena so we can load data.
    // The subscriber is stored inside the SubBufferedEntry in the arena.
    // We need to reach it through the arena.
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let sub_ptr = unsafe { arena_ptr.add(meta.offset) } as *const MockSubscriber;

    // Load CDR-encoded TestMsg(42) into the subscriber
    let (data, len) = encode_test_msg(42);
    unsafe { &*sub_ptr }.load(data, len);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 1);
    assert!(result.any_work());
    assert_eq!(*received.lock().unwrap(), Some(42));
}

// ====================================================================
// Borrowed (zero-copy) subscription E2E (Phase 229.6, issue 0007)
//
// These hand-written types mirror EXACTLY what the codegen emits for a
// `{ uint32 width; uint8[] pixels; float32[] ranges; }` message with
// `pixels` + `ranges` in `borrowed` mode (golden test:
// rosidl-codegen test_nros_borrowed_mode_view_and_marker /
// test_nros_borrowed_float_sequence_uses_le_view). The test drives the
// full owned-publish-wire → borrowed-subscribe path through `spin_once`,
// proving the generated shape compiles + decodes against the runtime.
// ====================================================================

/// Owned message — the publish side (matches codegen `Image`).
#[derive(Debug, Clone, PartialEq)]
struct Image {
    width: u32,
    pixels: heapless::Vec<u8, 64>,
    ranges: heapless::Vec<f32, 64>,
}

impl Serialize for Image {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u32(self.width)?;
        writer.write_u32(self.pixels.len() as u32)?;
        for b in &self.pixels {
            writer.write_u8(*b)?;
        }
        writer.write_u32(self.ranges.len() as u32)?;
        for f in &self.ranges {
            writer.write_f32(*f)?;
        }
        Ok(())
    }
}

/// Borrowed view — the zero-copy receive side (matches codegen `ImageView<'a>`).
struct ImageView<'a> {
    width: u32,
    pixels: &'a [u8],
    ranges: LeSliceView<'a, f32>,
}

impl<'a> DeserializeView<'a> for ImageView<'a> {
    fn deserialize_view(reader: &mut CdrReader<'a>) -> Result<Self, DeserError> {
        Ok(Self {
            width: reader.read_u32()?,
            pixels: reader.read_slice_u8()?,
            ranges: reader.read_le_slice::<f32>()?,
        })
    }
}

/// ZST marker — matches codegen `ImageViewable`.
struct ImageViewable;
impl ViewableMessage for ImageViewable {
    type View<'a> = ImageView<'a>;
    const TYPE_NAME: &'static str = "test/msg/Image";
    const TYPE_HASH: &'static str = "image_hash";
}

#[test]
fn borrowed_subscription_e2e_zero_copy_through_spin_once() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    let nid = executor.node_builder("borrowed_e2e").build().unwrap();

    // Captured on the receive side: (width, pixels copy, ranges decoded).
    type Captured = (u32, std::vec::Vec<u8>, std::vec::Vec<f32>);
    let received: std::sync::Arc<std::sync::Mutex<Option<Captured>>> =
        std::sync::Arc::new(std::sync::Mutex::new(None));
    let received2 = received.clone();

    executor
        .node_mut(nid)
        .create_subscription_viewable::<ImageViewable, _>("/image", move |view: &ImageView<'_>| {
            *received2.lock().unwrap() = Some((
                view.width,
                view.pixels.to_vec(),
                view.ranges.iter().collect(),
            ));
        })
        .unwrap();

    // Encode the OWNED message exactly as a publisher would, then feed those
    // wire bytes to the borrowed subscriber's mock handle.
    let msg = Image {
        width: 7,
        pixels: heapless::Vec::from_slice(&[10, 20, 30, 40, 250]).unwrap(),
        ranges: heapless::Vec::from_slice(&[1.5f32, -2.25, 3.0e10]).unwrap(),
    };
    let mut buf = [0u8; 256];
    let len = {
        let mut w = CdrWriter::new_with_header(&mut buf).unwrap();
        msg.serialize(&mut w).unwrap();
        w.position()
    };

    // The MockSubscriber is the first field of SubBufferedViewEntry, so it
    // sits at the entry offset (same layout trick as the typed test above).
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let sub_ptr = unsafe { arena_ptr.add(meta.offset) } as *const MockSubscriber;
    unsafe { &*sub_ptr }.load(buf, len);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 1);
    assert!(result.any_work());

    let got = received.lock().unwrap().take().expect("callback fired");
    assert_eq!(got.0, 7);
    assert_eq!(got.1, std::vec![10, 20, 30, 40, 250]);
    assert_eq!(got.2, std::vec![1.5f32, -2.25, 3.0e10]);
}

#[test]
fn test_multiple_subscriptions() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_multiple_subscriptions")
        .build()
        .unwrap();
    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count1 = count.clone();
    let count2 = count.clone();

    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/topic1", move |_msg: &TestMsg| {
            count1.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/topic2", move |_msg: &TestMsg| {
            count2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // Load data into both subscribers
    let (data, len) = encode_test_msg(10);
    let meta0 = executor.entries[0].as_ref().unwrap();
    let meta1 = executor.entries[1].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe { &*(arena_ptr.add(meta0.offset) as *const MockSubscriber) }.load(data, len);
    let (data2, len2) = encode_test_msg(20);
    unsafe { &*(arena_ptr.add(meta1.offset) as *const MockSubscriber) }.load(data2, len2);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 2);
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 2);
}

/// Phase 110.B — when two subscriptions are bound to `Edf` SCs,
/// the one with the earlier deadline dispatches first regardless of
/// registration order.
#[test]
fn test_edf_dispatch_order() {
    use crate::executor::sched_context::{DeadlinePolicy, OptUs, SchedClass, SchedContext};
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // `firing_order` records the data field of every msg the callbacks
    // see, in dispatch order.
    let firing_order = std::sync::Arc::new(std::sync::Mutex::new(std::vec::Vec::<i32>::new()));
    let order_late = firing_order.clone();
    let order_early = firing_order.clone();

    let nid = executor
        .node_builder("test_edf_dispatch_order")
        .build()
        .unwrap();
    // Registered first → has lower DescIdx → would normally dispatch
    // first under the FIFO path. Bind to a *later* deadline.
    let h_late = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/late", move |msg: &TestMsg| {
            order_late.lock().unwrap().push(msg.data);
        })
        .unwrap();

    // Registered second → higher DescIdx → would dispatch second under
    // FIFO. Bind to an *earlier* deadline so EDF promotes it.
    let h_early = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/early", move |msg: &TestMsg| {
            order_early.lock().unwrap().push(msg.data);
        })
        .unwrap();

    let sc_late = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Edf,
            deadline_us: OptUs::from_us(1000),
            deadline_policy: DeadlinePolicy::Activated,
            ..Default::default()
        })
        .unwrap();
    let sc_early = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Edf,
            deadline_us: OptUs::from_us(100),
            deadline_policy: DeadlinePolicy::Activated,
            ..Default::default()
        })
        .unwrap();

    executor
        .bind_handle_to_sched_context(h_late, sc_late)
        .unwrap();
    executor
        .bind_handle_to_sched_context(h_early, sc_early)
        .unwrap();

    // Load data into both subscribers — `data` field identifies which
    // is which in the firing log.
    let (d_late, n_late) = encode_test_msg(10);
    let (d_early, n_early) = encode_test_msg(20);
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off_late = executor.entries[0].as_ref().unwrap().offset;
    let off_early = executor.entries[1].as_ref().unwrap().offset;
    unsafe { &*(arena_ptr.add(off_late) as *const MockSubscriber) }.load(d_late, n_late);
    unsafe { &*(arena_ptr.add(off_early) as *const MockSubscriber) }.load(d_early, n_early);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 2);

    let order = firing_order.lock().unwrap();
    // Earlier-deadline (data=20) must precede later-deadline (data=10).
    assert_eq!(*order, std::vec![20, 10]);
}

/// Phase 110.F — an `os_pri`-bound callback is dispatched.
///
/// phase-359 W10 — read what this covers NOW. The worker pool moved off
/// `std::thread` onto the platform task ABI, so it exists only where a platform
/// provider is linked (`rmw-cffi`, the same proxy `node_wake` uses) — and this
/// module is compiled `not(feature = "rmw-cffi")`, because a real backend would
/// displace `MockSession`. So in THIS configuration there is no pool and the
/// assertion below rides the cooperative fallback.
///
/// That is worth stating rather than leaving the old "via worker" wording to
/// imply coverage that is no longer here: the fallback is what this proves, and
/// the worker path now needs a platform-linked build to exercise. Registering
/// the dispatcher still works everywhere, which is the other half of the
/// contract and is what keeps this test meaningful.
///
/// Uses a no-op `apply_policy` (non-root tests can't lift to SCHED_FIFO).
#[cfg(feature = "scheduler-os-priority")]
#[test]
fn test_os_priority_worker_dispatches_callback() {
    use crate::executor::sched_context::{SchedClass, SchedContext};
    use nros_platform_api::SchedPolicy;
    fn apply_noop(_p: SchedPolicy) -> Result<(), nros_platform_api::SchedError> {
        Ok(())
    }
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor.register_os_priority_dispatcher(apply_noop);
    let nid = executor
        .node_builder("test_os_priority_worker_dispatches_callback")
        .build()
        .unwrap();

    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count_cb = count.clone();
    let h = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/picas", move |_msg: &TestMsg| {
            count_cb.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    let sc_id = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Fifo,
            os_pri: 1,
            ..Default::default()
        })
        .unwrap();
    executor.bind_handle_to_sched_context(h, sc_id).unwrap();

    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off = executor.entries[0].as_ref().unwrap().offset;
    let (d, n) = encode_test_msg(7);
    unsafe { &*(arena_ptr.add(off) as *const MockSubscriber) }.load(d, n);

    // No pool in this configuration (see the doc comment), so the dispatch is
    // synchronous on the cooperative path; the sleep remains only to keep the
    // assertion valid if a pool ever IS present here.
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    std::thread::sleep(std::time::Duration::from_millis(50));

    assert_eq!(
        count.load(std::sync::atomic::Ordering::SeqCst),
        1,
        "os_pri-bound callback must dispatch — via a worker where the platform \
         hosts one, cooperatively where it does not"
    );
}

/// Phase 110.G — TT-window gate suppresses dispatch when the
/// current monotonic time falls outside `[off, off + duration)`.
/// Coexists with the existing class-based dispatch — this test uses
/// a `Fifo` SC with a TT window set, demonstrating that the gate is
/// orthogonal to class.
#[test]
fn test_tt_window_gate_suppresses_outside_window() {
    use crate::executor::sched_context::{OptUs, SchedClass, SchedContext};
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // Window = [50ms..51ms) within a 60-second major frame.
    // Test runs in a single spin_once well under 50 ms after the
    // executor's epoch — phase < 50 ms → outside window → dispatch
    // suppressed.
    executor.register_time_triggered_dispatcher(60_000_000);
    let nid = executor
        .node_builder("test_tt_window_gate_suppresses_outside_window")
        .build()
        .unwrap();

    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count_cb = count.clone();
    let h = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/tt", move |_msg: &TestMsg| {
            count_cb.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // Far-future window so the spin happens outside it.
    let sc_id = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Fifo,
            tt_window_offset_us: OptUs::from_us(50_000_000),
            tt_window_duration_us: OptUs::from_us(1_000),
            ..Default::default()
        })
        .unwrap();
    executor.bind_handle_to_sched_context(h, sc_id).unwrap();

    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off = executor.entries[0].as_ref().unwrap().offset;
    let (d, n) = encode_test_msg(1);
    unsafe { &*(arena_ptr.add(off) as *const MockSubscriber) }.load(d, n);

    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        count.load(std::sync::atomic::Ordering::SeqCst),
        0,
        "TT window gate must suppress dispatch outside the active slot"
    );
}

/// Phase 110.G — schedule-table builder declares + applies a full
/// cyclic schedule in one call: validates window layout, sets
/// major-frame length, creates one SC per window with the right
/// TT-gate fields. Two-window schedule with the first window
/// covering [0..1s) within a 2-second major frame ensures the
/// test's spin (well under 1s after executor construction) fires
/// the entry bound to window-0 and suppresses the one bound to
/// window-1.
#[test]
fn test_time_triggered_dispatch_active_window() {
    use crate::executor::sched_context::{TimeTriggeredSchedule, TimeTriggeredWindow};

    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let count_w0 = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count_w1 = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let cb0 = count_w0.clone();
    let cb1 = count_w1.clone();
    let nid = executor
        .node_builder("test_apply_time_triggered_schedule")
        .build()
        .unwrap();
    let h0 = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/tt0", move |_msg: &TestMsg| {
            cb0.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();
    let h1 = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/tt1", move |_msg: &TestMsg| {
            cb1.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    let schedule = TimeTriggeredSchedule::<2>::new_full(
        2_000_000,
        [
            TimeTriggeredWindow::new(0, 1_000_000, "w0"),
            TimeTriggeredWindow::new(1_000_000, 1_000_000, "w1"),
        ],
    );
    let ids = executor
        .apply_time_triggered_schedule(&schedule)
        .expect("schedule should validate");
    executor.bind_handle_to_sched_context(h0, ids[0]).unwrap();
    executor.bind_handle_to_sched_context(h1, ids[1]).unwrap();

    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off0 = executor.entries[0].as_ref().unwrap().offset;
    let off1 = executor.entries[1].as_ref().unwrap().offset;
    let (d0, n0) = encode_test_msg(10);
    let (d1, n1) = encode_test_msg(20);
    unsafe { &*(arena_ptr.add(off0) as *const MockSubscriber) }.load(d0, n0);
    unsafe { &*(arena_ptr.add(off1) as *const MockSubscriber) }.load(d1, n1);

    let _ = executor.spin_once(core::time::Duration::from_millis(0));

    // Phase < 1s → only the entry bound to window-0 fires; the
    // entry bound to window-1 must stay suppressed.
    assert_eq!(
        count_w0.load(std::sync::atomic::Ordering::SeqCst),
        1,
        "entry bound to window-0 should dispatch inside its active slot"
    );
    assert_eq!(
        count_w1.load(std::sync::atomic::Ordering::SeqCst),
        0,
        "entry bound to window-1 must stay suppressed outside its slot"
    );
}

/// Phase 110.G — schedule validation. Overlapping windows are an
/// authoring bug and surface as a structured error rather than
/// silent precedence between dispatchers.
#[test]
fn test_time_triggered_schedule_rejects_overlapping_windows() {
    use crate::executor::sched_context::{
        TimeTriggeredSchedule, TimeTriggeredScheduleError, TimeTriggeredWindow,
    };

    let bad = TimeTriggeredSchedule::<2>::new_full(
        1_000_000,
        [
            TimeTriggeredWindow::new(0, 600_000, "w0"),
            TimeTriggeredWindow::new(500_000, 200_000, "w1"),
        ],
    );
    let err = bad.validate().unwrap_err();
    assert!(
        matches!(err, TimeTriggeredScheduleError::WindowsOverlap { .. }),
        "overlapping windows must surface as a WindowsOverlap error, got {err:?}"
    );

    let oversize =
        TimeTriggeredSchedule::<1>::new_full(1_000, [TimeTriggeredWindow::new(500, 600, "w0")]);
    let err = oversize.validate().unwrap_err();
    assert!(
        matches!(
            err,
            TimeTriggeredScheduleError::WindowExceedsMajorFrame { .. }
        ),
        "window past major-frame end must surface as WindowExceedsMajorFrame, got {err:?}"
    );
}

/// Phase 110.E — `SchedClass::Sporadic` budget suppression. After the
/// budget is exhausted within a period, the bound subscription's
/// callback no longer fires until the next period boundary refills
/// the budget.
#[test]
fn test_sporadic_budget_exhaustion_suppresses_dispatch() {
    use crate::executor::sched_context::{OptUs, SchedClass, SchedContext};
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_sporadic_budget_exhaustion")
        .build()
        .unwrap();
    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count_cb = count.clone();
    let h = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/sporadic", move |_msg: &TestMsg| {
            count_cb.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            // issue 0736 — the callback must COST something, because callback
            // runtime is now the only thing that consumes budget. This test
            // used to exhaust the budget by sleeping BETWEEN spins, which
            // worked only because the executor charged the inter-spin
            // wall-clock gap to every sporadic SC whether or not it had run.
            // That was the defect; a test that relies on it is asserting the
            // bug. Sleeping HERE is the same test of the same rule — "an SC
            // that has spent its budget is not dispatched again" — expressed
            // in the quantity the budget actually bounds.
            std::thread::sleep(std::time::Duration::from_millis(2));
        })
        .unwrap();

    // 1 us budget per 60 s period: one dispatch costs ~2 ms of measured
    // runtime, which exhausts it for the rest of the period.
    let sc_id = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Sporadic,
            budget_us: OptUs::from_us(1),
            period_us: OptUs::from_us(60_000_000),
            ..Default::default()
        })
        .unwrap();
    executor.bind_handle_to_sched_context(h, sc_id).unwrap();

    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off = executor.entries[0].as_ref().unwrap().offset;

    // Cycle 1 — refill puts 1 us in the budget, the callback fires and spends
    // ~2 ms of it. The period is 60 s, so nothing refills before cycle 2.
    let (d, n) = encode_test_msg(1);
    unsafe { &*(arena_ptr.add(off) as *const MockSubscriber) }.load(d, n);
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        count.load(std::sync::atomic::Ordering::SeqCst),
        1,
        "cycle 1 must dispatch — otherwise cycle 2 proves nothing"
    );

    // Cycle 2 — budget is 0; dispatch must be suppressed.
    let initial = count.load(std::sync::atomic::Ordering::SeqCst);
    let (d, n) = encode_test_msg(2);
    unsafe { &*(arena_ptr.add(off) as *const MockSubscriber) }.load(d, n);
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    let after = count.load(std::sync::atomic::Ordering::SeqCst);

    // Strictly assert no new dispatch on cycle 2.
    assert_eq!(
        after, initial,
        "Sporadic SC must suppress dispatch when budget exhausted"
    );
}

/// Phase 110.E.b follow-up — per-callback runtime accounting.
/// When a Sporadic SC has an `AtomicSporadicState` registered (the
/// ISR-driven refill path), `spin_once` measures each bound
/// callback's wall-clock dispatch time + `consume`s those microseconds
/// from the atomic budget. Replaces the cycle-level over-attribution
/// that previously charged the full cycle `delta_us` against every
/// Sporadic SC regardless of which entries actually fired.
#[test]
#[cfg(feature = "alloc")]
fn test_atomic_sporadic_per_callback_runtime_consumed() {
    use crate::executor::{
        sched_context::{OptUs, SchedClass, SchedContext},
        spin::OpaqueTimerHandle,
    };

    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_atomic_sporadic_per_callback_runtime_consumed")
        .build()
        .unwrap();
    // Subscription that sleeps a known interval so the per-callback
    // dispatch timing is deterministic enough to assert on.
    let h = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/timed", move |_msg: &TestMsg| {
            std::thread::sleep(std::time::Duration::from_millis(10));
        })
        .unwrap();

    // Sporadic SC with 1 s budget — plenty so dispatch always
    // proceeds; the assertion is on the *consumed* amount, not
    // suppression.
    let sc_id = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Sporadic,
            budget_us: OptUs::from_us(1_000_000),
            period_us: OptUs::from_us(60_000_000),
            ..Default::default()
        })
        .unwrap();
    executor.bind_handle_to_sched_context(h, sc_id).unwrap();

    // Build a no-op `OpaqueTimerHandle` so `register_sporadic_timer`
    // accepts the call — the test doesn't need a real periodic
    // refill; it only needs the `AtomicSporadicState` slot wired so
    // the dispatcher consumes runtime from it.
    extern "C" fn noop_destroy(_handle: *mut core::ffi::c_void) {}
    let fake_timer = unsafe { OpaqueTimerHandle::new(core::ptr::null_mut(), noop_destroy) };
    let state = executor.register_sporadic_timer(sc_id, fake_timer).unwrap();
    let before = state
        .budget_remaining_us
        .load(portable_atomic::Ordering::Acquire);

    // Drive one dispatch — the registered closure sleeps 10 ms.
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off = executor.entries[0].as_ref().unwrap().offset;
    let (d, n) = encode_test_msg(7);
    unsafe { &*(arena_ptr.add(off) as *const MockSubscriber) }.load(d, n);
    let _ = executor.spin_once(core::time::Duration::from_millis(0));

    let after = state
        .budget_remaining_us
        .load(portable_atomic::Ordering::Acquire);

    // Per-callback runtime consumed at least the 10 ms (= 10_000 us)
    // sleep, but well under the full 1 s budget — proves dispatch-
    // local measurement, not cycle-level over-attribution.
    let consumed = before.saturating_sub(after);
    assert!(
        consumed >= 10_000,
        "expected at least 10 ms (10000 us) consumed for a 10 ms callback, got {consumed} us"
    );
    assert!(
        consumed < 500_000,
        "consumed {consumed} us suggests cycle-level over-attribution, not per-callback measurement"
    );
}

/// Phase 110.E.b — overrun detection. A Sporadic-bound callback
/// whose measured runtime exceeds the SC's budget bumps
/// `AtomicSporadicState::overrun_count` + stamps `last_overrun_us`.
/// The "oneshot-IRQ-and-cancel" pattern from the design doc is
/// structurally equivalent for cooperative single-thread dispatch
/// (we can't preempt a running callback), so this is the
/// diagnostic signal end-callers consume to tune budgets.
#[test]
#[cfg(feature = "alloc")]
fn test_atomic_overrun_exceeds_budget() {
    use crate::executor::{
        sched_context::{OptUs, SchedClass, SchedContext},
        spin::OpaqueTimerHandle,
    };

    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_atomic_sporadic_overrun_recorded")
        .build()
        .unwrap();
    // Subscription sleeps 25 ms; budget is 5 ms → must overrun.
    let h = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/overrun", move |_msg: &TestMsg| {
            std::thread::sleep(std::time::Duration::from_millis(25));
        })
        .unwrap();

    let sc_id = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Sporadic,
            budget_us: OptUs::from_us(5_000), // 5 ms budget
            period_us: OptUs::from_us(60_000_000),
            ..Default::default()
        })
        .unwrap();
    executor.bind_handle_to_sched_context(h, sc_id).unwrap();

    extern "C" fn noop_destroy(_h: *mut core::ffi::c_void) {}
    let fake_timer = unsafe { OpaqueTimerHandle::new(core::ptr::null_mut(), noop_destroy) };
    let state = executor.register_sporadic_timer(sc_id, fake_timer).unwrap();
    assert_eq!(
        state.overrun_count.load(portable_atomic::Ordering::Acquire),
        0
    );

    // Drive one dispatch — the registered closure sleeps 25 ms.
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off = executor.entries[0].as_ref().unwrap().offset;
    let (d, n) = encode_test_msg(42);
    unsafe { &*(arena_ptr.add(off) as *const MockSubscriber) }.load(d, n);
    let _ = executor.spin_once(core::time::Duration::from_millis(0));

    let count = state.overrun_count.load(portable_atomic::Ordering::Acquire);
    let last = state
        .last_overrun_us
        .load(portable_atomic::Ordering::Acquire);
    assert_eq!(count, 1, "overrun_count must increment exactly once");
    // Overrun = measured - budget; measured ≥ 25 ms, budget = 5 ms.
    // last_overrun_us should be ≥ 20 ms = 20_000 us.
    assert!(
        last >= 20_000,
        "last_overrun_us {last} should be ≥ 20000 (25 ms callback - 5 ms budget)"
    );

    // `clear_overrun_stats` resets both counters.
    state.clear_overrun_stats();
    assert_eq!(
        state.overrun_count.load(portable_atomic::Ordering::Acquire),
        0
    );
    assert_eq!(
        state
            .last_overrun_us
            .load(portable_atomic::Ordering::Acquire),
        0
    );
}

/// Phase 110.D — multi-executor smoke test. Spawns two Executors,
/// each on its own OS thread with a different `SchedPolicy`. Mirrors
/// the shape of the drone S1 / watchdog S3 acceptance scenarios from
/// the phase doc. Live SCHED_FIFO requires `CAP_SYS_NICE`, so the
/// test uses a no-op `apply_policy` and only asserts the lifecycle
/// works — full timing acceptance for S1 / S3 ships once the
/// integration harness with privileged scheduling is in place.
#[test]
fn test_open_threaded_two_executors_independent_lifecycle() {
    use nros_platform_api::SchedPolicy;
    fn apply_noop(_p: SchedPolicy) -> Result<(), nros_platform_api::SchedError> {
        Ok(())
    }

    // Critical executor — would run at SCHED_FIFO os_pri 90 in a
    // privileged process.
    let crit = executor_with_clock(MockSession::new());
    let crit_handle = unsafe {
        crit.open_threaded(
            SchedPolicy::Fifo { os_pri: 90 },
            apply_noop,
            core::time::Duration::from_millis(1),
        )
    };

    // BE executor — would run at SCHED_FIFO os_pri 10 in a
    // privileged process.
    let be = executor_with_clock(MockSession::new());
    let be_handle = unsafe {
        be.open_threaded(
            SchedPolicy::Fifo { os_pri: 10 },
            apply_noop,
            core::time::Duration::from_millis(5),
        )
    };

    // Let both run a couple of cycles.
    std::thread::sleep(std::time::Duration::from_millis(20));

    // Halt each independently — order shouldn't matter.
    assert!(crit_handle.join().is_ok());
    assert!(be_handle.join().is_ok());
}

/// Phase 110.D.b — smoke test for `Executor::open_threaded`. Spawns
/// the executor onto a fresh OS thread, lets it spin, then halts via
/// the returned `ThreadHandle`.
#[test]
fn test_open_threaded_spawn_and_halt() {
    use nros_platform_api::SchedPolicy;
    let session = MockSession::new();
    let executor: Executor = executor_with_clock(session);

    // Apply-policy fn that always succeeds — running as a non-root
    // unit test we can't actually lift to SCHED_FIFO, so the
    // smoke-test just exercises the spawn / halt / join lifecycle.
    fn apply_noop(_p: SchedPolicy) -> Result<(), nros_platform_api::SchedError> {
        Ok(())
    }

    // SAFETY: `from_session` (Owned) Executor is Send-correct;
    // `unsafe impl Send for Executor` covers it unconditionally.
    let handle = unsafe {
        executor.open_threaded(
            SchedPolicy::Fifo { os_pri: 1 },
            apply_noop,
            core::time::Duration::from_millis(1),
        )
    };

    // Let the executor thread run a couple of spin cycles.
    std::thread::sleep(std::time::Duration::from_millis(20));

    // Halt + join must complete within a generous bound.
    let join_res = handle.join();
    assert!(join_res.is_ok());
}

/// Phase 110.C — `Critical`-bucket callback runs before
/// `BestEffort`-bucket callback when both are ready in the same cycle,
/// regardless of registration order.
#[test]
fn test_bucketed_priority_dispatch_order() {
    use crate::executor::sched_context::{Priority, SchedClass, SchedContext};
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let firing_order = std::sync::Arc::new(std::sync::Mutex::new(std::vec::Vec::<i32>::new()));
    let o_be = firing_order.clone();
    let o_crit = firing_order.clone();

    let nid = executor
        .node_builder("test_bucketed_priority_dispatch_order")
        .build()
        .unwrap();
    // Registered first (lower DescIdx) — bound to BestEffort.
    let h_be = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/be", move |msg: &TestMsg| {
            o_be.lock().unwrap().push(msg.data);
        })
        .unwrap();
    // Registered second — bound to Critical so the bucket promotion
    // beats registration order.
    let h_crit = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/crit", move |msg: &TestMsg| {
            o_crit.lock().unwrap().push(msg.data);
        })
        .unwrap();

    let sc_be = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Fifo,
            priority: Priority::BestEffort,
            ..Default::default()
        })
        .unwrap();
    let sc_crit = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Fifo,
            priority: Priority::Critical,
            ..Default::default()
        })
        .unwrap();
    executor.bind_handle_to_sched_context(h_be, sc_be).unwrap();
    executor
        .bind_handle_to_sched_context(h_crit, sc_crit)
        .unwrap();

    let (d_be, n_be) = encode_test_msg(1);
    let (d_crit, n_crit) = encode_test_msg(2);
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off_be = executor.entries[0].as_ref().unwrap().offset;
    let off_crit = executor.entries[1].as_ref().unwrap().offset;
    unsafe { &*(arena_ptr.add(off_be) as *const MockSubscriber) }.load(d_be, n_be);
    unsafe { &*(arena_ptr.add(off_crit) as *const MockSubscriber) }.load(d_crit, n_crit);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 2);

    let order = firing_order.lock().unwrap();
    // Critical (data=2) drains before BestEffort (data=1).
    assert_eq!(*order, std::vec![2, 1]);
}

/// Phase 110.B — default `Fifo` SC binding preserves registration
/// order even when other entries are bound to `Edf` SCs.
#[test]
fn test_fifo_default_binding_preserved_alongside_edf() {
    use crate::executor::sched_context::{OptUs, SchedClass, SchedContext};
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let firing_order = std::sync::Arc::new(std::sync::Mutex::new(std::vec::Vec::<i32>::new()));
    let o1 = firing_order.clone();
    let o2 = firing_order.clone();

    let nid = executor
        .node_builder("test_fifo_default_binding_preserved_alongside_edf")
        .build()
        .unwrap();
    let _h1 = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/fifo1", move |msg: &TestMsg| {
            o1.lock().unwrap().push(msg.data);
        })
        .unwrap();
    let h2 = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/edf", move |msg: &TestMsg| {
            o2.lock().unwrap().push(msg.data);
        })
        .unwrap();

    let sc_edf = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Edf,
            deadline_us: OptUs::from_us(50),
            ..Default::default()
        })
        .unwrap();
    executor.bind_handle_to_sched_context(h2, sc_edf).unwrap();

    let (d1, n1) = encode_test_msg(1);
    let (d2, n2) = encode_test_msg(2);
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off1 = executor.entries[0].as_ref().unwrap().offset;
    let off2 = executor.entries[1].as_ref().unwrap().offset;
    unsafe { &*(arena_ptr.add(off1) as *const MockSubscriber) }.load(d1, n1);
    unsafe { &*(arena_ptr.add(off2) as *const MockSubscriber) }.load(d2, n2);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 2);

    // EDF-bound entry (data=2) drains first; FIFO-bound (data=1) second.
    let order = firing_order.lock().unwrap();
    assert_eq!(*order, std::vec![2, 1]);
}

#[test]
fn test_arena_overflow() {
    let session = MockSession::new();
    // Arena is ARENA_SIZE bytes (derived to fit MAX_CBS worst-case ActionClient
    // entries — see `nros-node/build.rs`). Use a subscription RX buffer larger
    // than `ARENA_SIZE / MAX_CBS` so we exhaust the arena before running out
    // of entry slots. Each SubBufferedEntry holds a triple buffer (3 × RX_BUF)
    // plus a per-entry header, so an RX buffer of `ARENA_SIZE / 4` triggers
    // overflow well before `MAX_CBS` registrations.
    const OVERFLOW_RX_BUF: usize = crate::config::ARENA_SIZE / 4;
    let mut executor = executor_with_clock(session);
    let nid = executor
        .node_builder("test_arena_overflow")
        .build()
        .unwrap();

    let topics = ["/a", "/b", "/c", "/d"];
    let mut filled = 0;
    for topic in &topics {
        let result = executor
            .node_mut(nid)
            .subscription(topic)
            .qos(QoSProfile::default().keep_last(1))
            .typed::<TestMsg>()
            .rx_buffer::<OVERFLOW_RX_BUF>()
            .build(|_msg: &TestMsg| {});
        if result.is_err() {
            break;
        }
        filled += 1;
    }

    // We should have been able to add at least 1 but not all 4 (arena too small).
    assert!(filled >= 1, "Should fit at least 1 large subscription");
    assert!(
        filled < 4,
        "Arena should overflow before 4 large subscriptions, got {filled}"
    );

    // Verify the next add fails with BufferTooSmall.
    let result = executor
        .node_mut(nid)
        .subscription("/overflow")
        .qos(QoSProfile::default().keep_last(1))
        .typed::<TestMsg>()
        .rx_buffer::<OVERFLOW_RX_BUF>()
        .build(|_msg: &TestMsg| {});
    assert_eq!(result, Err(NodeError::BufferTooSmall));
}

#[test]
fn test_entry_slots_exhausted() {
    // Issue #545 — `MAX_CBS` is a BUILD-TIME knob (`NROS_EXECUTOR_MAX_CBS`,
    // also reachable via `$DOTCONFIG`), and cargo reads `.cargo/config.toml`
    // from the current directory UPWARD — so a workspace that vendors
    // nano-ros and raises the knob for its own image raises it here too.
    // Assume nothing: fill exactly `MAX_CBS` slots, then assert the next
    // one is refused. Small rx buffers so the slot table, not the arena,
    // is the bound being tested.
    let session = MockSession::new();
    let mut executor = executor_with_clock(session);
    let nid = executor
        .node_builder("test_entry_slots_exhausted")
        .build()
        .unwrap();

    for i in 0..crate::config::MAX_CBS {
        let topic = alloc::format!("/slot{i}");
        let r = executor
            .node_mut(nid)
            .subscription(&topic)
            .qos(QoSProfile::default().keep_last(1))
            .typed::<TestMsg>()
            .rx_buffer::<64>()
            .build(|_msg: &TestMsg| {});
        assert!(
            r.is_ok(),
            "registration {i} of MAX_CBS={} failed with {r:?} — if this is \
             BufferTooSmall the ARENA is the binding constraint, not the \
             slot table, and this test needs splitting (see #545)",
            crate::config::MAX_CBS
        );
    }

    // One past the table. Issue 0095: distinct `ExecutorFull` (capacity),
    // not the generic `BufferTooSmall`.
    let result = executor
        .node_mut(nid)
        .subscription("/one-too-many")
        .qos(QoSProfile::default().keep_last(1))
        .typed::<TestMsg>()
        .rx_buffer::<64>()
        .build(|_msg: &TestMsg| {});
    assert_eq!(result, Err(NodeError::ExecutorFull));
}

#[test]
fn test_spin_once_result_counts() {
    let result = SpinOnceResult::new();
    assert!(!result.any_work());
    assert!(!result.any_errors());
    assert_eq!(result.total(), 0);
    assert_eq!(result.total_errors(), 0);

    let result = SpinOnceResult {
        subscriptions_processed: 2,
        timers_fired: 1,
        services_handled: 1,
        subscription_errors: 0,
        service_errors: 0,
    };
    assert!(result.any_work());
    assert!(!result.any_errors());
    assert_eq!(result.total(), 4);
}

#[test]
fn test_drop_runs_without_panic() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_drop_runs_without_panic")
        .build()
        .unwrap();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_msg: &TestMsg| {})
        .unwrap();

    // executor drops here — Drop impl must not panic
}

#[test]
fn test_executor_spin_once_no_entries() {
    // Executor with no registered callbacks — spin_once just calls drive_io.
    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(!result.any_work());
}

#[test]
fn test_arena_alignment() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_arena_alignment")
        .build()
        .unwrap();
    // Add a subscription, then check the offset is properly aligned
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_msg: &TestMsg| {})
        .unwrap();

    let meta = executor.entries[0].as_ref().unwrap();
    let entry_align = core::mem::align_of::<arena::SubBufferedEntry<TestMsg, fn(&TestMsg)>>();
    assert_eq!(meta.offset % entry_align, 0);
}

// ====================================================================
// Timer callback tests
// ====================================================================

#[test]
fn test_add_timer_and_fire() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count2 = count.clone();
    executor
        .register_timer(TimerDuration::from_millis(100), move || {
            count2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // Not enough time elapsed — should not fire
    let result = elapse_then_spin_once(&mut executor, 50);
    assert_eq!(result.timers_fired, 0);
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 0);

    // Now enough time elapsed (50 + 60 = 110 >= 100)
    let result = elapse_then_spin_once(&mut executor, 60);
    assert_eq!(result.timers_fired, 1);
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 1);
}

/// phase-425 W4 — a `TimerClockSource::Ros` timer follows the SIMULATED clock,
/// and a wall timer on the same executor does not.
///
/// One test rather than four, deliberately: the ROS-time override is
/// process-global (one simulated clock per image, as in Rust rclrs), so four
/// tests would race each other inside the one test binary.
///
/// Merging them handles the four; it does not handle the OTHER tests that touch
/// the same globals, which is issue 1104. `SimTimeGuard` does that -- it orders
/// this against them and restores the override on the way out, replacing the
/// hand-written clear this test used to end with.
#[test]
fn ros_time_timer_follows_the_simulated_clock() {
    let _sim_time = SimTimeGuard::acquire();
    use nros_core::clock::Clock;
    use std::sync::atomic::{AtomicUsize, Ordering};

    const MS: i64 = 1_000_000;

    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // A simulator is publishing /clock: t = 10 s.
    Clock::set_ros_time_override(10_000 * MS);

    let ros_ticks = std::sync::Arc::new(AtomicUsize::new(0));
    let wall_ticks = std::sync::Arc::new(AtomicUsize::new(0));
    let r = ros_ticks.clone();
    let w = wall_ticks.clone();

    executor
        .register_timer_on_clock(
            TimerDuration::from_millis(100),
            super::arena::TimerClockSource::Ros,
            move || {
                r.fetch_add(1, Ordering::SeqCst);
            },
        )
        .unwrap();
    executor
        .register_timer(TimerDuration::from_millis(100), move || {
            w.fetch_add(1, Ordering::SeqCst);
        })
        .unwrap();

    // 1. PAUSED. Real time passes; simulated time does not. The wall timer
    //    fires and the ROS timer must not -- this is the whole point.
    for _ in 0..3 {
        let _ = elapse_then_spin_once(&mut executor, 120);
    }
    assert_eq!(
        ros_ticks.load(Ordering::SeqCst),
        0,
        "a ROS-time timer fired while the simulator was paused: 360 ms of REAL \
         time passed and /clock did not move, so no period elapsed"
    );
    assert!(
        wall_ticks.load(Ordering::SeqCst) >= 3,
        "the wall timer on the same executor must be unaffected by the pause \
         (that is what makes it the right clock for a watchdog); fired {} times",
        wall_ticks.load(Ordering::SeqCst)
    );

    // 2. RATE. Simulated time advances 100 ms per spin while only ~10 ms of real
    //    time passes: four ROS activations, and the wall timer stays behind.
    let wall_before = wall_ticks.load(Ordering::SeqCst);
    for i in 1..=4 {
        Clock::set_ros_time_override(10_000 * MS + i * 100 * MS);
        let _ = elapse_then_spin_once(&mut executor, 10);
    }
    assert_eq!(
        ros_ticks.load(Ordering::SeqCst),
        4,
        "four 100 ms steps of simulated time on a 100 ms ROS timer must be four \
         activations -- the timer tracks /clock, not the executor's spin delta"
    );
    assert!(
        wall_ticks.load(Ordering::SeqCst) - wall_before <= 1,
        "~40 ms of real time cannot be four activations of a 100 ms wall timer"
    );

    // 3. BACKWARDS JUMP -- a bag looping, or a simulator reset. The period
    //    restarts; the timer must not stall for the length of the jump.
    Clock::set_ros_time_override(1_000 * MS); // back 9.4 s
    let _ = elapse_then_spin_once(&mut executor, 1);
    assert_eq!(
        ros_ticks.load(Ordering::SeqCst),
        4,
        "a backwards jump is not an elapsed period"
    );
    Clock::set_ros_time_override(1_100 * MS); // one period past the new origin
    let _ = elapse_then_spin_once(&mut executor, 1);
    assert_eq!(
        ros_ticks.load(Ordering::SeqCst),
        5,
        "after a backwards jump the NEXT period must fire on schedule; a timer \
         that accumulated the negative delta would stay dead for 9.4 s"
    );
}

/// phase-425 W3b — `use_sim_time` is a SWITCH, not a value, and declaring it
/// attaches the `/clock` source the way rclcpp's does.
///
/// The reconciliation, not the wire: a MockSession has no publisher to receive
/// from, so what this asserts is that the subscription gets installed at the
/// right moment and that the gate follows the parameter. The sample path is
/// covered by `time_source::tests` and the timer behaviour by
/// `ros_time_timer_follows_the_simulated_clock`.
#[cfg(all(feature = "sim-time", feature = "param-services"))]
#[test]
fn use_sim_time_attaches_and_detaches_the_clock_source() {
    let _sim_time = SimTimeGuard::acquire();
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // Declared BEFORE any node exists, which is the order a generated entry
    // uses: `nros::main!` emits `apply_param_services` ahead of its per-node
    // `register` calls.
    executor.declare_parameter("use_sim_time", nros_params::ParameterValue::Bool(true));
    assert!(
        !executor.ros_time_source_installed(),
        "declaring the parameter must not subscribe on the spot — there is no \
         node yet to hang the subscription on"
    );

    // A spin with no node must not install and must not give up either: the
    // request stays pending. This is the ordering the reconcile exists for.
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(
        !executor.ros_time_source_installed(),
        "there is still no node; installing would mean subscribing against a \
         node table entry that does not exist"
    );

    drop(executor.create_node("sim_node").expect("create node"));
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(
        executor.ros_time_source_installed(),
        "the first spin after the request must install the /clock source; \
         a request that never reconciles is a parameter that does nothing"
    );
    assert!(
        crate::time_source::is_active(),
        "installed and not armed would drop every sample"
    );

    // Idempotent: a second spin must not add a second subscription. A fresh
    // registration would take a new slot, so a stable handle is the evidence.
    let handle = executor.ros_time_source_handle();
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        executor.ros_time_source_handle(),
        handle,
        "reconciliation ran twice and subscribed twice"
    );

    // Turning it off stops SAMPLES. The subscription stays — the executor has
    // no entity removal — so the gate is what must change.
    executor.declare_parameter("use_sim_time", nros_params::ParameterValue::Bool(false));
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(
        !crate::time_source::is_active(),
        "use_sim_time went false and /clock samples would still be installed"
    );

    // The global and the override are put back by `SimTimeGuard`'s drop, which
    // restores what it observed instead of asserting a default.
}

/// phase-425 W3b — a non-bool `use_sim_time` attaches nothing.
///
/// ROS 2 lets a node declare the name with a wrong type; what must NOT happen is
/// a time source attaching off a value that does not mean "yes".
#[cfg(all(feature = "sim-time", feature = "param-services"))]
#[test]
fn a_non_bool_use_sim_time_attaches_nothing() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    executor.declare_parameter("use_sim_time", nros_params::ParameterValue::Integer(1));
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(
        !executor.ros_time_source_installed(),
        "an INTEGER 1 is not `true`; attaching off it would make the switch \
         answer to any truthy-looking value"
    );
}

/// Issue 1036 — arena exhaustion must REACH the boot record, not merely return
/// an error.
///
/// The issue's own resolution rests on the record being "unit-tested on the
/// host". Those tests exercise the RECORD (`boot_report::tests` — layout,
/// magic, monotonic stage). Nothing tested the LINK: that the allocator, on
/// failing, actually writes the number an operator would dump. That link is the
/// whole instrument — on the board this was written for, the console UART is
/// not wired, so the record is the only channel, and a record nobody writes to
/// is indistinguishable from the silence it was built to replace.
///
/// `arena_alloc_with_trailing` specifically, because that is the half that was
/// silent (issue 0900 fixed the sibling and left this one), and it is the half
/// carrying every buffered subscription and every action entry — what an island
/// image actually allocates.
///
/// The record is a process-global static and only the FIRST failure is kept
/// (`compare_exchange` from 0), so this test must be the ONLY one in its
/// process that records a failure. It is not named `boot_report*` for exactly
/// that reason: `boot_report::tests` has its own `note_alloc_failed(100, 8)`
/// case, and a shared filter put both in one binary — measured, this assertion
/// caught it reading `(100, 8)`. `just check boot-report-tests` runs the two
/// groups as SEPARATE cargo invocations, which is two processes and therefore
/// two records.
#[cfg(nros_boot_report)]
#[test]
fn arena_exhaustion_reaches_the_boot_record() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // Untouched to start with, which is also the assertion that nothing else in
    // this run got here first — if it fails, the filter let a sibling in and
    // every number below would be that sibling's.
    let before = crate::boot_report::snapshot();
    assert_eq!(
        (before.failed_alloc_size, before.failed_alloc_shortfall),
        (0, 0),
        "another test recorded an allocation failure before this one; the \
         record keeps only the FIRST, so this run cannot attribute it"
    );

    // One byte more than the arena can hold, asked for through the path that
    // used to say nothing.
    let capacity = executor.arena.len();
    let ask = capacity + 1;
    let err = executor
        .arena_alloc_with_trailing::<u8>(ask)
        .expect_err("an allocation larger than the whole arena must fail");
    assert!(
        matches!(err, NodeError::BufferTooSmall),
        "unexpected error for an over-large arena request: {err:?}"
    );

    let after = crate::boot_report::snapshot();
    assert!(
        after.failed_alloc_size > 0,
        "the arena refused an allocation and the boot record says nothing. On a \
         board with no console this record is the ONLY channel, so a zero here \
         is the original defect with an instrument bolted on."
    );
    // The SHORTFALL is the number an operator adds to the knob, so it has to be
    // the real difference rather than a flag that something failed.
    assert!(
        after.failed_alloc_shortfall > 0,
        "size recorded but shortfall zero: the record names a failure without \
         naming what would fix it, and the shortfall is the actionable half"
    );
    assert!(
        u64::from(after.failed_alloc_shortfall) <= ask as u64,
        "shortfall {} exceeds the request {ask}, which cannot be a difference \
         against a non-negative capacity",
        after.failed_alloc_shortfall
    );

    // And the record must survive a SECOND failure unchanged: the first
    // allocation to fail is the one that explains the boot, and a later,
    // larger, incidental failure overwriting it would replace the cause with a
    // symptom.
    let _ = executor.arena_alloc_with_trailing::<u8>(ask * 4);
    let again = crate::boot_report::snapshot();
    assert_eq!(
        (again.failed_alloc_size, again.failed_alloc_shortfall),
        (after.failed_alloc_size, after.failed_alloc_shortfall),
        "a second failure overwrote the first; the record must keep the \
         allocation that explains the boot, not the last one to be attempted"
    );
}

#[test]
fn test_timer_repeats() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count2 = count.clone();
    executor
        .register_timer(TimerDuration::from_millis(100), move || {
            count2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // Fire 3 times
    let _ = elapse_then_spin_once(&mut executor, 100);
    let _ = elapse_then_spin_once(&mut executor, 100);
    let _ = elapse_then_spin_once(&mut executor, 100);
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 3);
}

/// phase-417 W5.c — `timer_is_ready` / `timer_elapsed_us` must report what the
/// DISPATCHER is about to do, not a second opinion about it.
///
/// These two exist because the C accessors `nros_timer_is_ready` and
/// `nros_timer_get_time_since_last_call` were filed as gaps: the state was
/// already in the arena and nothing exposed it. The risk in exposing it is a
/// readiness answer that drifts from `arena::timer_try_process`'s own guard,
/// so the assertion here is agreement — not-yet-due, due, and due-again after
/// a fire — plus the `None` that a non-timer handle must produce, since a
/// `false` there would report a live timer as merely not ready.
#[test]
fn timer_readiness_and_elapsed_agree_with_the_dispatcher() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // Bounds, not equalities, on the elapsed readings: `spin_once` credits the
    // REAL wall-clock cost of the spin on top of the injected delta (see its
    // note on the timer delta), so a driven timer runs a little ahead of the
    // nominal schedule. The lower bound is the assertion — an upper-bound-only
    // check passes a counter that never advanced.
    let id = executor
        .register_timer(TimerDuration::from_millis(100), || {})
        .unwrap();

    assert_eq!(executor.timer_elapsed_us(id), Some(0));
    assert_eq!(executor.timer_is_ready(id), Some(false));

    // Short of the period: elapsed advances, readiness does not.
    let result = elapse_then_spin_once(&mut executor, 40);
    assert_eq!(result.timers_fired, 0);
    let elapsed = executor.timer_elapsed_us(id).unwrap();
    assert!(
        (40_000..50_000).contains(&elapsed),
        "one 40ms step: {elapsed}us"
    );
    assert_eq!(executor.timer_is_ready(id), Some(false));

    let result = elapse_then_spin_once(&mut executor, 40);
    assert_eq!(result.timers_fired, 0, "80ms is short of the 100ms period");
    let elapsed = executor.timer_elapsed_us(id).unwrap();
    assert!(
        (80_000..95_000).contains(&elapsed),
        "two 40ms steps: {elapsed}us"
    );
    assert_eq!(executor.timer_is_ready(id), Some(false));

    let result = elapse_then_spin_once(&mut executor, 40);
    assert_eq!(result.timers_fired, 1, "120ms >= 100ms period");
    // The activation rewound `elapsed_us` by one period, and that rewind is
    // exactly what "time since last call" means: the reading must drop below
    // the period rather than keep climbing.
    let elapsed = executor.timer_elapsed_us(id).unwrap();
    assert!(
        (20_000..40_000).contains(&elapsed),
        "the fire must rewind by one period: {elapsed}us"
    );
    assert_eq!(executor.timer_is_ready(id), Some(false));

    // A cancelled timer is never ready, however much time has passed.
    executor.cancel_timer(id).unwrap();
    let _ = elapse_then_spin_once(&mut executor, 500);
    assert_eq!(executor.timer_is_ready(id), Some(false));
    assert!(executor.timer_is_canceled(id));

    // A handle that is not a timer has no answer, which is not `false`.
    let not_a_timer = HandleId(id.0 + 1);
    assert_eq!(executor.timer_elapsed_us(not_a_timer), None);
    assert_eq!(executor.timer_is_ready(not_a_timer), None);
}

#[test]
fn test_timer_oneshot_fires_once() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count2 = count.clone();
    executor
        .register_timer_oneshot(TimerDuration::from_millis(50), move || {
            count2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // First spin fires
    let result = elapse_then_spin_once(&mut executor, 60);
    assert_eq!(result.timers_fired, 1);
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 1);

    // Second spin should NOT fire again
    let result = elapse_then_spin_once(&mut executor, 60);
    assert_eq!(result.timers_fired, 0);
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 1);
}

#[test]
fn test_timer_does_not_fire_at_zero_delta() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count2 = count.clone();
    executor
        .register_timer(TimerDuration::from_millis(100), move || {
            count2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // Zero delta should never fire
    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.timers_fired, 0);
}

#[test]
fn test_timer_with_subscriptions() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let timer_count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let timer_count2 = timer_count.clone();
    executor
        .register_timer(TimerDuration::from_millis(100), move || {
            timer_count2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    let nid = executor
        .node_builder("test_timer_with_subscriptions")
        .build()
        .unwrap();
    let sub_count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let sub_count2 = sub_count.clone();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", move |_msg: &TestMsg| {
            sub_count2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // Load data into subscription
    let (data, len) = encode_test_msg(99);
    let meta1 = executor.entries[1].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe { &*(arena_ptr.add(meta1.offset) as *const MockSubscriber) }.load(data, len);

    let result = elapse_then_spin_once(&mut executor, 100);
    assert_eq!(result.timers_fired, 1);
    assert_eq!(result.subscriptions_processed, 1);
    assert_eq!(timer_count.load(std::sync::atomic::Ordering::SeqCst), 1);
    assert_eq!(sub_count.load(std::sync::atomic::Ordering::SeqCst), 1);
}

// ====================================================================
// Action types for testing
// ====================================================================

#[derive(Debug, Clone, Default, PartialEq)]
struct TestGoal {
    order: i32,
}

impl RosMessage for TestGoal {
    const TYPE_NAME: &'static str = "test/action/TestAction_Goal";
    const TYPE_HASH: &'static str = "test_hash";
}

// phase-380 W4 — was `#[cfg(rmw_needs_type_descriptors)]`: the schema is
// now required by `MessageForRmw` on EVERY backend, because that is where
// a subscription's build-time size bound comes from.
impl nros_serdes::schema::Message for TestGoal {
    const TYPE_NAME: &'static str = "test/action/TestAction_Goal";
    const FIELDS: &'static [nros_serdes::schema::Field] = &[nros_serdes::schema::Field {
        name: "order",
        ty: nros_serdes::schema::FieldType::Int32,
        offset: 0,
    }];
}

impl Serialize for TestGoal {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.order)
    }
}

impl Deserialize for TestGoal {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            order: reader.read_i32()?,
        })
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
struct TestResult {
    value: i32,
}

impl RosMessage for TestResult {
    const TYPE_NAME: &'static str = "test/action/TestAction_Result";
    const TYPE_HASH: &'static str = "test_hash";
}

// phase-380 W4 — was `#[cfg(rmw_needs_type_descriptors)]`: the schema is
// now required by `MessageForRmw` on EVERY backend, because that is where
// a subscription's build-time size bound comes from.
impl nros_serdes::schema::Message for TestResult {
    const TYPE_NAME: &'static str = "test/action/TestAction_Result";
    const FIELDS: &'static [nros_serdes::schema::Field] = &[nros_serdes::schema::Field {
        name: "value",
        ty: nros_serdes::schema::FieldType::Int32,
        offset: 0,
    }];
}

impl Serialize for TestResult {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.value)
    }
}

impl Deserialize for TestResult {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            value: reader.read_i32()?,
        })
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
struct TestFeedback {
    progress: i32,
}

impl RosMessage for TestFeedback {
    const TYPE_NAME: &'static str = "test/action/TestAction_Feedback";
    const TYPE_HASH: &'static str = "test_hash";
}

// phase-380 W4 — was `#[cfg(rmw_needs_type_descriptors)]`: the schema is
// now required by `MessageForRmw` on EVERY backend, because that is where
// a subscription's build-time size bound comes from.
impl nros_serdes::schema::Message for TestFeedback {
    const TYPE_NAME: &'static str = "test/action/TestAction_Feedback";
    const FIELDS: &'static [nros_serdes::schema::Field] = &[nros_serdes::schema::Field {
        name: "progress",
        ty: nros_serdes::schema::FieldType::Int32,
        offset: 0,
    }];
}

impl Serialize for TestFeedback {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.progress)
    }
}

impl Deserialize for TestFeedback {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            progress: reader.read_i32()?,
        })
    }
}

struct TestAction;

impl RosAction for TestAction {
    type Goal = TestGoal;
    type Result = TestResult;
    type Feedback = TestFeedback;
    // For tests the envelope types reuse the inner message types: the executor
    // tests only exercise the spin-arena registration path, not on-wire CDR
    // round-trips of the action service shapes.
    type SendGoalRequest = TestGoal;
    type SendGoalResponse = TestResult;
    type GetResultRequest = TestGoal;
    type GetResultResponse = TestResult;
    type FeedbackMessage = TestFeedback;
    const ACTION_NAME: &'static str = "test/action/dds_/TestAction_";
    const ACTION_HASH: &'static str = "test_hash";
}

// ====================================================================
// Action server tests
// ====================================================================

#[test]
fn test_add_action_server_registers() {
    let session = MockSession::new();
    // Use small buffers to fit within the 4096-byte arena.
    let mut executor = executor_with_clock(session);

    let handle = executor
        .register_action_server_sized::<TestAction, _, _, 64, 64, 64, 1>(
            "/test_action",
            |_goal_id, _goal: &TestGoal| nros_core::GoalResponse::AcceptAndExecute,
            |_id: &nros_core::GoalId, _status: nros_core::GoalStatus| {
                nros_core::CancelResponse::Accept
            },
        )
        .unwrap();

    // Verify the entry was registered
    assert!(executor.entries[0].is_some());
    assert_eq!(handle.entry_index, 0);
}

#[test]
fn test_action_server_spin_once_no_requests() {
    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    let _handle = executor
        .register_action_server_sized::<TestAction, _, _, 64, 64, 64, 1>(
            "/test_action",
            |_goal_id, _goal: &TestGoal| nros_core::GoalResponse::AcceptAndExecute,
            |_id: &nros_core::GoalId, _status: nros_core::GoalStatus| {
                nros_core::CancelResponse::Accept
            },
        )
        .unwrap();

    // With no pending requests, spin_once should return no work
    let result = executor.spin_once(core::time::Duration::from_millis(10));
    assert_eq!(result.services_handled, 0);
    assert!(!result.any_work());
}

#[test]
fn test_action_server_registers_and_spins() {
    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    let _server_handle = executor
        .register_action_server_sized::<TestAction, _, _, 64, 64, 64, 1>(
            "/test_action",
            |_goal_id, _goal: &TestGoal| nros_core::GoalResponse::AcceptAndExecute,
            |_id: &nros_core::GoalId, _status: nros_core::GoalStatus| {
                nros_core::CancelResponse::Accept
            },
        )
        .unwrap();

    // Action server registered
    assert!(executor.entries[0].is_some());

    let result = executor.spin_once(core::time::Duration::from_millis(10));
    assert!(!result.any_work());
}

#[test]
fn test_drop_with_mixed_entries() {
    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_drop_with_mixed_entries")
        .build()
        .unwrap();
    // Register one of each kind — use small buffers to fit in 4096-byte arena.
    executor
        .node_mut(nid)
        .subscription("/sub")
        .qos(QoSProfile::default().keep_last(1))
        .typed::<TestMsg>()
        .rx_buffer::<64>()
        .build(|_msg: &TestMsg| {})
        .unwrap();
    executor
        .register_timer(TimerDuration::from_millis(100), || {})
        .unwrap();
    let _server = executor
        .register_action_server_sized::<TestAction, _, _, 64, 64, 64, 1>(
            "/act",
            |_goal_id, _goal: &TestGoal| nros_core::GoalResponse::AcceptAndExecute,
            |_id: &nros_core::GoalId, _status: nros_core::GoalStatus| {
                nros_core::CancelResponse::Accept
            },
        )
        .unwrap();

    // Drop must clean up all 3 entries without panicking
}

// ====================================================================
// spin_one_period tests (no_std)
// ====================================================================

#[test]
fn test_spin_one_period_remaining_time() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // elapsed < period → remaining = period - elapsed
    let r = executor.spin_one_period(100, 30);
    assert_eq!(r.remaining_ms, 70);
    assert_eq!(r.work.total(), 0);
}

#[test]
fn test_spin_one_period_overrun() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // elapsed > period → remaining saturates to 0
    let r = executor.spin_one_period(10, 50);
    assert_eq!(r.remaining_ms, 0);
}

#[test]
fn test_spin_one_period_exact() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // elapsed == period → remaining = 0
    let r = executor.spin_one_period(42, 42);
    assert_eq!(r.remaining_ms, 0);
}

#[test]
fn test_spin_options_default() {
    let opts = SpinOptions::default();
    assert!(opts.timeout.is_none());
    assert!(!opts.only_next);
    assert!(opts.max_callbacks.is_none());
}

#[test]
fn test_spin_options_builders() {
    let opts = SpinOptions::new()
        .timeout(core::time::Duration::from_secs(5))
        .max_callbacks(10);
    assert_eq!(opts.timeout, Some(core::time::Duration::from_secs(5)));
    assert_eq!(opts.max_callbacks, Some(10));
    assert!(!opts.only_next);

    let opts_once = SpinOptions::spin_once();
    assert!(opts_once.only_next);
}

// ====================================================================
// std-gated spin tests
// ====================================================================

#[test]
fn test_spin_blocking_only_next() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // only_next exits after single iteration
    let result = executor.spin_blocking(SpinOptions::spin_once());
    assert!(result.is_ok());
}

#[test]
fn test_spin_blocking_halt() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // Pre-set cancel flag → exits immediately. `cancel()`, not the deprecated
    // `halt()`: phase-417 W4.c renamed it to rclcpp's word, and `-D warnings`
    // makes the old spelling a hard error rather than a warning.
    executor.cancel();
    assert!(executor.is_halted());

    // spin_blocking resets halt then checks it — so we need a thread
    let halt = executor.halt_flag();
    std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(50));
        halt.store(true, std::sync::atomic::Ordering::SeqCst);
    });
    let result = executor.spin_blocking(SpinOptions::default());
    assert!(result.is_ok());
}

/// issue 0709 — a timeout this build cannot measure is an ERROR, not "forever".
///
/// The clock is cleared directly rather than through a config, because
/// `ExecutorConfig`'s timing fields deliberately cannot clobber the platform
/// default with `None` (issue 0671): "not specified" and "there is none" are
/// different, and only a build with no clock source reaches the second. This
/// test synthesises that build.
///
/// It is the regression test for a ten-hour hang: `spin_blocking` used to read
/// "no clock" as "no deadline" and loop until halted, in the one API whose
/// contract is "returns after N ms".
#[test]
fn spin_blocking_with_a_timeout_and_no_clock_errors() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor.clock_us_fn = None;

    let err = executor
        .spin_blocking(SpinOptions::new().timeout(core::time::Duration::from_millis(50)))
        .expect_err("a timeout with no clock must fail, not spin forever");
    assert_eq!(err, NodeError::NotInitialized);
}

/// …and an UNTIMED `spin_blocking` still runs, because "until halt" is a
/// promise a clockless build can keep. Halted from a peer thread so the test
/// cannot hang if the guard is ever widened by mistake.
#[test]
fn spin_blocking_without_a_timeout_still_runs_with_no_clock() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor.clock_us_fn = None;

    let halt = executor.halt_flag();
    std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(50));
        halt.store(true, std::sync::atomic::Ordering::SeqCst);
    });
    assert!(executor.spin_blocking(SpinOptions::new()).is_ok());
}

/// issue 0709 — the sibling guard: a period that cannot be paced is an error
/// rather than a busy-loop pretending to run at `period`.
#[test]
fn spin_period_with_no_clock_errors() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor.clock_us_fn = None;

    let err = executor
        .spin_period(core::time::Duration::from_millis(10))
        .expect_err("a period with no clock must fail, not busy-loop");
    assert_eq!(err, NodeError::NotInitialized);
}

/// issue 0709 — `from_session_with` is the seam `from_session` never had: a
/// caller that brings its own session can bring its own clock.
///
/// Asserted with `spin_once`, which is BOUNDED — it does one round and
/// returns. Three earlier versions of this test observed the clock through
/// `spin_blocking`/`spin_one_period_timed` instead and hung under the parallel
/// harness; a stub clock and a loop that re-reads a clock are a bad pair, and
/// the guard those versions were reaching for is already covered by the two
/// tests above. What is left to prove here is only that the constructor
/// INSTALLS what it is given, and a bounded call proves it without a loop.
#[test]
fn from_session_with_installs_the_callers_clock() {
    fn fake_clock() -> u64 {
        static TICKS: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
        TICKS.fetch_add(1_000, core::sync::atomic::Ordering::SeqCst)
    }

    let cfg = ExecutorConfig::default().clock_us(fake_clock);
    let mut executor: Executor = Executor::from_session_with(MockSession::new(), &cfg);
    // The clock is installed, so a timed spin is no longer refused by the
    // issue-0709 guard — and one bounded round is enough to reach it.
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(
        executor.clock_us_fn.is_some(),
        "from_session_with must install the config's clock"
    );
    assert!(
        executor.clock_us_fn.map(|c| c()).is_some(),
        "and it must be callable"
    );
}

#[test]
fn test_spin_blocking_timeout() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let start = std::time::Instant::now();
    let result =
        executor.spin_blocking(SpinOptions::new().timeout(core::time::Duration::from_millis(50)));
    assert!(result.is_ok());
    // Should exit within a reasonable time after 50ms timeout
    assert!(start.elapsed() < std::time::Duration::from_secs(2));
}

#[test]
fn test_spin_one_period_timed_no_overrun() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let period = std::time::Duration::from_millis(50);
    let result = executor.spin_one_period_timed(period);
    // Mock session returns instantly, so no overrun
    assert!(!result.overrun);
    assert_eq!(result.work.total(), 0);
}

#[test]
fn test_halt_flag_clone() {
    let session = MockSession::new();
    let executor: Executor = executor_with_clock(session);

    let flag = executor.halt_flag();
    assert!(!executor.is_halted());

    flag.store(true, std::sync::atomic::Ordering::SeqCst);
    assert!(executor.is_halted());
}

#[test]
fn test_spin_period_halts() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let halt = executor.halt_flag();
    std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(50));
        halt.store(true, std::sync::atomic::Ordering::SeqCst);
    });

    let result = executor.spin_period(std::time::Duration::from_millis(10));
    assert!(result.is_ok());
}

#[test]
fn test_wake_handle_clone() {
    let session = MockSession::new();
    let executor: Executor = executor_with_clock(session);

    let wake = executor.wake_handle();
    assert!(!wake.load(std::sync::atomic::Ordering::SeqCst));

    executor.wake();
    assert!(wake.load(std::sync::atomic::Ordering::SeqCst));
}

#[test]
fn test_wake_cleared_each_spin() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // Pre-arm the flag — spin_once must swap-clear it.
    executor.wake();
    let wake = executor.wake_handle();
    assert!(wake.load(std::sync::atomic::Ordering::SeqCst));

    let _ = executor.spin_once(core::time::Duration::from_millis(1));
    assert!(
        !wake.load(std::sync::atomic::Ordering::SeqCst),
        "spin_once must consume the wake flag"
    );
}

#[test]
fn test_halt_raises_wake_flag() {
    let session = MockSession::new();
    let executor: Executor = executor_with_clock(session);

    let wake = executor.wake_handle();
    assert!(!wake.load(std::sync::atomic::Ordering::SeqCst));

    executor.cancel();
    assert!(executor.is_halted());
    assert!(
        wake.load(std::sync::atomic::Ordering::SeqCst),
        "halt() must also set the wake flag so an in-flight spin_once \
         falls through to the halt check on its next iteration"
    );
}

#[test]
fn test_guard_handle_send_across_thread() {
    // Phase 124.B.7.d — GuardCondition must be Send (so a
    // worker thread / signal handler can own it and call trigger()).
    // Sync impl assertion via thread move and rejoin.
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let (_id, handle) = executor
        .register_guard_condition(|| {})
        .expect("register_guard_condition");

    let t = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(5));
        handle.trigger();
        // Returning ownership-of-nothing here proves the handle
        // moved into the thread; if it weren't Send the compiler
        // would have rejected this.
    });
    t.join().unwrap();
    // No assert on wake_flag here — gated on rmw-cffi feature; this
    // test runs with the default feature set.
}

#[test]
fn test_wake_short_circuits_drive_timeout() {
    // Pre-arming wake_flag should make spin_once skip its blocking
    // wait on drive_io (timeout collapses to 0) and return promptly,
    // even when the caller asked for a 200ms tick.
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    executor.wake();

    let start = std::time::Instant::now();
    let _ = executor.spin_once(core::time::Duration::from_millis(200));
    let elapsed = start.elapsed();
    assert!(
        elapsed < core::time::Duration::from_millis(50),
        "wake_flag set → spin_once must not wait 200ms; elapsed = {elapsed:?}",
    );
}

// ====================================================================
// Phase 49: HandleId / HandleSet / ReadinessSnapshot tests
// ====================================================================

#[test]
fn test_handle_id_from_add_subscription() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_handle_id_from_add_subscription")
        .build()
        .unwrap();
    let id = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/a", |_msg: &TestMsg| {})
        .unwrap();
    assert_eq!(id, HandleId(0));

    let id2 = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/b", |_msg: &TestMsg| {})
        .unwrap();
    assert_eq!(id2, HandleId(1));
}

#[test]
fn test_handle_set_operations() {
    let a = HandleId(0);
    let b = HandleId(1);
    let c = HandleId(5);

    let set = a | b;
    assert!(set.contains(a));
    assert!(set.contains(b));
    assert!(!set.contains(c));
    assert_eq!(set.len(), 2);

    let set2 = set | c;
    assert!(set2.contains(c));
    assert_eq!(set2.len(), 3);

    let empty = HandleSet::EMPTY;
    assert!(empty.is_empty());
    assert_eq!(empty.len(), 0);
}

#[test]
fn test_handle_set_union() {
    let set1 = HandleSet::EMPTY.insert(HandleId(0)).insert(HandleId(2));
    let set2 = HandleSet::EMPTY.insert(HandleId(1)).insert(HandleId(2));
    let union = set1 | set2;
    assert!(union.contains(HandleId(0)));
    assert!(union.contains(HandleId(1)));
    assert!(union.contains(HandleId(2)));
    assert_eq!(union.len(), 3);
}

#[test]
fn test_readiness_snapshot() {
    let snap = ReadinessSnapshot {
        bits: 0b101,
        count: 3,
    };
    assert!(snap.is_ready(HandleId(0)));
    assert!(!snap.is_ready(HandleId(1)));
    assert!(snap.is_ready(HandleId(2)));
    assert_eq!(snap.ready_count(), 2);
    assert_eq!(snap.total(), 3);

    let set = HandleId(0) | HandleId(2);
    assert!(snap.all_ready(set));
    assert!(snap.any_ready(set));

    let set2 = HandleId(0) | HandleId(1);
    assert!(!snap.all_ready(set2));
    assert!(snap.any_ready(set2));
}

// ====================================================================
// Phase 49: Trigger condition tests
// ====================================================================

#[test]
fn test_trigger_any_fires_on_data() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor.set_trigger(Trigger::Any);
    let nid = executor
        .node_builder("test_trigger_any_fires_on_data")
        .build()
        .unwrap();

    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_msg: &TestMsg| {})
        .unwrap();

    // Load data
    let (data, len) = encode_test_msg(1);
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe { &*(arena_ptr.add(meta.offset) as *const MockSubscriber) }.load(data, len);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 1);
}

#[test]
fn test_trigger_any_no_data_no_dispatch() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor.set_trigger(Trigger::Any);
    let nid = executor
        .node_builder("test_trigger_any_no_data_no_dispatch")
        .build()
        .unwrap();

    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_msg: &TestMsg| {})
        .unwrap();

    // No data loaded → trigger should not pass (for subscriptions)
    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 0);
}

#[test]
fn test_trigger_always_fires_without_data() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor.set_trigger(Trigger::Always);
    let nid = executor
        .node_builder("test_trigger_always_fires_without_data")
        .build()
        .unwrap();

    let called = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let called2 = called.clone();
    let id = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", move |_msg: &TestMsg| {
            called2.store(true, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // Set invocation to Always so callback fires even without data
    executor.set_invocation(id, InvocationMode::Always);

    // No data, but trigger Always → dispatch phase runs, callback fires
    let _result = executor.spin_once(core::time::Duration::from_millis(0));
    // Subscription take returns None, so subscriptions_processed stays 0
    // but the callback IS invoked (Always invocation) — try_process returns Ok(false)
    // because there's no actual data
    assert!(!called.load(std::sync::atomic::Ordering::SeqCst));
}

#[test]
fn test_trigger_one_fires_on_specific_handle() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_trigger_one_fires_on_specific_handle")
        .build()
        .unwrap();
    let _id0 = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/topic0", |_: &TestMsg| {})
        .unwrap();
    let id1 = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/topic1", |_: &TestMsg| {})
        .unwrap();

    executor.set_trigger(Trigger::One(id1));

    // Load data only on topic0 (not the trigger handle)
    let (data, len) = encode_test_msg(1);
    let meta0 = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe { &*(arena_ptr.add(meta0.offset) as *const MockSubscriber) }.load(data, len);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    // Trigger requires handle 1 to have data, but only handle 0 does
    assert_eq!(result.subscriptions_processed, 0);

    // Now load data on topic1
    let (data2, len2) = encode_test_msg(2);
    let meta1 = executor.entries[1].as_ref().unwrap();
    unsafe { &*(arena_ptr.add(meta1.offset) as *const MockSubscriber) }.load(data2, len2);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(result.subscriptions_processed >= 1);
}

#[test]
fn test_trigger_predicate() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_trigger_predicate")
        .build()
        .unwrap();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_: &TestMsg| {})
        .unwrap();

    // Custom predicate that requires at least 1 ready handle
    executor.set_trigger(Trigger::Predicate(|snap: &ReadinessSnapshot| {
        snap.ready_count() >= 1
    }));

    // No data → predicate returns false
    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 0);
}

// ====================================================================
// Phase 49: Guard condition tests
// ====================================================================

#[test]
fn test_guard_condition_trigger_fires_callback() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let called = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let called2 = called.clone();

    let (_id, handle) = executor
        .register_guard_condition(move || {
            called2.store(true, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // Not triggered yet
    let _result = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(!called.load(std::sync::atomic::Ordering::SeqCst));

    // Trigger the guard condition
    handle.trigger();

    let _result = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(called.load(std::sync::atomic::Ordering::SeqCst));
}

#[test]
fn test_guard_condition_clears_after_trigger() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count2 = count.clone();

    let (_id, handle) = executor
        .register_guard_condition(move || {
            count2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    // Trigger once
    handle.trigger();
    executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 1);

    // Without re-triggering, callback should not fire again
    executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 1);

    // Trigger again
    handle.trigger();
    executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 2);
}

// ====================================================================
// Phase 49: Raw subscription callback tests
// ====================================================================

#[test]
fn test_raw_subscription_callback() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    static RAW_CALLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    static RAW_LEN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

    unsafe extern "C" fn raw_cb(_data: *const u8, len: usize, _context: *mut core::ffi::c_void) {
        RAW_CALLED.store(true, std::sync::atomic::Ordering::SeqCst);
        RAW_LEN.store(len, std::sync::atomic::Ordering::SeqCst);
    }

    RAW_CALLED.store(false, std::sync::atomic::Ordering::SeqCst);

    let _id = executor
        .add_arena_subscription_c_callback::<{ crate::config::DEFAULT_RX_BUF_SIZE }>(
            None,
            "/test",
            "test/msg/TestMsg",
            "test_hash",
            QoSProfile::default().keep_last(1),
            raw_cb,
            core::ptr::null_mut(),
            None, // no group
            // phase-402 W2 — no hint in this test; 0 = no opinion.
            0,
        )
        .unwrap();

    // Load CDR data into the mock subscriber
    let (data, len) = encode_test_msg(99);
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe {
        let sub_ptr = arena_ptr.add(meta.offset) as *const MockSubscriber;
        (*sub_ptr).load(data, len);
    }

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 1);
    assert!(RAW_CALLED.load(std::sync::atomic::Ordering::SeqCst));
    assert_eq!(RAW_LEN.load(std::sync::atomic::Ordering::SeqCst), len);
}

#[test]
fn test_raw_subscription_info_callback() {
    // Phase 189.M3.4 — the C-fn-ptr-with-attachment subscription path.
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    static INFO_CALLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    static INFO_LEN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    static INFO_ATT_LEN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

    unsafe extern "C" fn info_cb(
        _data: *const u8,
        len: usize,
        _att: *const u8,
        att_len: usize,
        _ctx: *mut core::ffi::c_void,
    ) {
        INFO_CALLED.store(true, std::sync::atomic::Ordering::SeqCst);
        INFO_LEN.store(len, std::sync::atomic::Ordering::SeqCst);
        INFO_ATT_LEN.store(att_len, std::sync::atomic::Ordering::SeqCst);
    }

    let _id = executor
        .add_arena_subscription_c_info_callback::<{ crate::config::DEFAULT_RX_BUF_SIZE }>(
            None,
            "/test",
            "test/msg/TestMsg",
            "test_hash",
            QoSProfile::default().keep_last(1),
            info_cb,
            core::ptr::null_mut(),
            0, // phase-408 W5a — no hint stated
        )
        .unwrap();

    let (data, len) = encode_test_msg(7);
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe {
        let sub_ptr = arena_ptr.add(meta.offset) as *const MockSubscriber;
        (*sub_ptr).load(data, len);
    }

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 1);
    assert!(INFO_CALLED.load(std::sync::atomic::Ordering::SeqCst));
    assert_eq!(INFO_LEN.load(std::sync::atomic::Ordering::SeqCst), len);
    // MockSubscriber has no native attachment ⇒ default 0-length attachment.
    assert_eq!(INFO_ATT_LEN.load(std::sync::atomic::Ordering::SeqCst), 0);
}

// ====================================================================
// phase-417 W5.a — TYPED C subscription delivery (caller-owned storage)
//
// rclc hands the callback a DESERIALISED message on a path with no
// allocator, by making the caller own the storage. These two tests pin
// both halves of that contract: the success path writes into the
// caller's struct and dispatches, and the failure path does NOT — it
// drops the sample and reports it, rather than handing the callback
// storage that still holds the previous message.
// ====================================================================

/// Caller-owned storage, the shape a generated `<Msg>` struct has.
#[repr(C)]
struct TypedProbeMsg {
    value: i32,
    /// Written by the deserialiser so the test can prove the CDR bytes
    /// reached it, not just that something was called.
    decoded_len: usize,
}

/// The erased form of a generated `<Msg>_deserialize`: writes into caller
/// storage, returns 0 / non-zero.
unsafe extern "C" fn typed_probe_deserialize(
    msg: *mut core::ffi::c_void,
    buffer: *const u8,
    buffer_size: usize,
) -> i32 {
    let bytes = unsafe { core::slice::from_raw_parts(buffer, buffer_size) };
    let Ok(mut reader) = CdrReader::new_with_header(bytes) else {
        return -1;
    };
    let Ok(value) = reader.read_i32() else {
        return -1;
    };
    let out = unsafe { &mut *(msg as *mut TypedProbeMsg) };
    out.value = value;
    out.decoded_len = buffer_size;
    0
}

/// A deserialiser that always refuses — stands in for a sample that does not
/// fit the caller's storage (a string longer than its declared bound refuses
/// exactly this way; `nros_cdr_read_string` errors on `str_len > max_len`
/// rather than truncating).
unsafe extern "C" fn typed_probe_deserialize_refuses(
    _msg: *mut core::ffi::c_void,
    _buffer: *const u8,
    _buffer_size: usize,
) -> i32 {
    -1
}

static TYPED_CB_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static TYPED_CB_VALUE: std::sync::atomic::AtomicI32 = std::sync::atomic::AtomicI32::new(0);
static TYPED_CB_LEN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static TYPED_CB_CTX_OK: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// The ported-rclc callback body: it reads FIELDS, and never CDR bytes.
unsafe extern "C" fn typed_probe_callback(
    msg: *const core::ffi::c_void,
    context: *mut core::ffi::c_void,
) {
    let m = unsafe { &*(msg as *const TypedProbeMsg) };
    TYPED_CB_CALLS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    TYPED_CB_VALUE.store(m.value, std::sync::atomic::Ordering::SeqCst);
    TYPED_CB_LEN.store(m.decoded_len, std::sync::atomic::Ordering::SeqCst);
    TYPED_CB_CTX_OK.store(!context.is_null(), std::sync::atomic::Ordering::SeqCst);
}

#[test]
fn typed_c_subscription_delivers_into_caller_storage() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    TYPED_CB_CALLS.store(0, std::sync::atomic::Ordering::SeqCst);
    TYPED_CB_VALUE.store(0, std::sync::atomic::Ordering::SeqCst);
    TYPED_CB_LEN.store(0, std::sync::atomic::Ordering::SeqCst);
    TYPED_CB_CTX_OK.store(false, std::sync::atomic::Ordering::SeqCst);

    // The caller owns this. Nothing in the executor allocates it, drops it, or
    // copies it — that is the whole mechanism.
    let mut storage = TypedProbeMsg {
        value: 0,
        decoded_len: 0,
    };
    let mut ctx: u32 = 0xC0FFEE;

    let _id = unsafe {
        executor.add_arena_subscription_c_typed_callback::<{ crate::config::DEFAULT_RX_BUF_SIZE }>(
            None,
            "/typed",
            "test/msg/TestMsg",
            "test_hash",
            QoSProfile::default().keep_last(1),
            core::ptr::NonNull::new(&mut storage as *mut TypedProbeMsg as *mut core::ffi::c_void)
                .unwrap(),
            typed_probe_deserialize,
            typed_probe_callback,
            &mut ctx as *mut u32 as *mut core::ffi::c_void,
            None,
            0,
        )
    }
    .unwrap();

    let (data, len) = encode_test_msg(4242);
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe {
        let sub_ptr = arena_ptr.add(meta.offset) as *const MockSubscriber;
        (*sub_ptr).load(data, len);
    }

    let result = executor.spin_once(core::time::Duration::from_millis(0));

    assert_eq!(result.subscriptions_processed, 1);
    assert_eq!(result.subscription_errors, 0);
    assert_eq!(
        TYPED_CB_CALLS.load(std::sync::atomic::Ordering::SeqCst),
        1,
        "the typed callback must be dispatched exactly once per sample"
    );
    assert_eq!(
        TYPED_CB_VALUE.load(std::sync::atomic::Ordering::SeqCst),
        4242,
        "the callback must see the DESERIALISED field, not CDR bytes"
    );
    assert_eq!(
        TYPED_CB_LEN.load(std::sync::atomic::Ordering::SeqCst),
        len,
        "the deserialiser must have been handed the whole sample"
    );
    assert!(
        TYPED_CB_CTX_OK.load(std::sync::atomic::Ordering::SeqCst),
        "the caller's context must reach the callback"
    );
    // ...and the CALLER's storage is what was written, which is the claim that
    // distinguishes this path from one that deserialises into an arena slot.
    assert_eq!(storage.value, 4242);
    assert_eq!(storage.decoded_len, len);
}

#[test]
fn typed_c_subscription_refused_decode_is_loud_and_undelivered() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    TYPED_CB_CALLS.store(0, std::sync::atomic::Ordering::SeqCst);

    // Poisoned on purpose: if the executor dispatched on a refused decode, the
    // callback would read THIS and report it as the message that just arrived.
    let mut storage = TypedProbeMsg {
        value: -1,
        decoded_len: 0,
    };

    let failures_before = super::arena::typed_deserialize_failures();

    let _id = unsafe {
        executor.add_arena_subscription_c_typed_callback::<{ crate::config::DEFAULT_RX_BUF_SIZE }>(
            None,
            "/typed_bad",
            "test/msg/TestMsg",
            "test_hash",
            QoSProfile::default().keep_last(1),
            core::ptr::NonNull::new(&mut storage as *mut TypedProbeMsg as *mut core::ffi::c_void)
                .unwrap(),
            typed_probe_deserialize_refuses,
            typed_probe_callback,
            core::ptr::null_mut(),
            None,
            0,
        )
    }
    .unwrap();

    let (data, len) = encode_test_msg(7);
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe {
        let sub_ptr = arena_ptr.add(meta.offset) as *const MockSubscriber;
        (*sub_ptr).load(data, len);
    }

    let result = executor.spin_once(core::time::Duration::from_millis(0));

    assert_eq!(
        TYPED_CB_CALLS.load(std::sync::atomic::Ordering::SeqCst),
        0,
        "a refused decode must NOT reach the callback — it would be handed the \
         previous message under the impression it was the new one"
    );
    assert_eq!(
        result.subscriptions_processed, 0,
        "a dropped sample is not processed work"
    );
    assert_eq!(
        result.subscription_errors, 1,
        "a refused decode must be OBSERVABLE — silence here is exactly what \
         RFC-0089 forbids"
    );
    assert!(
        result.any_errors(),
        "the spin result must report the failure"
    );
    // `>=`, not `==`. The counter is a PROCESS-GLOBAL atomic and cargo runs this
    // binary's tests on parallel threads, so between `failures_before` and here
    // any other test driving a typed subscription can bump it. Exact equality
    // asserts "no other test failed a decode", which is not this test's
    // business and is not something it can observe; the property under test is
    // that THIS refusal was counted. Exactness made it fail in `check::build`'s
    // parallel lane while passing solo.
    assert!(
        super::arena::typed_deserialize_failures() >= failures_before + 1,
        "the refusal must also be counted for the rate-limited nros_log report"
    );
    assert_eq!(
        storage.value, -1,
        "the caller's storage must be left as it was; a partial write the \
         callback never sees is still a write the NEXT dispatch could show"
    );
}

// ====================================================================
// Phase 49: Session borrowing tests
// ====================================================================

#[test]
fn test_from_session_ptr() {
    let mut session = MockSession::new();
    let executor: Executor = unsafe { Executor::from_session_ptr(&mut session) };

    // Session should be accessible
    let _session_ref = executor.session();
}

#[test]
fn test_from_session_ptr_create_node() {
    let mut session = MockSession::new();
    let mut executor: Executor = unsafe { Executor::from_session_ptr(&mut session) };

    let node = executor.create_node("test_node");
    assert!(node.is_ok());
}

// ====================================================================
// Phase 49: InvocationMode tests
// ====================================================================

#[test]
fn test_set_invocation_mode() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_set_invocation_mode")
        .build()
        .unwrap();
    let id = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_: &TestMsg| {})
        .unwrap();

    // Default is OnNewData
    assert_eq!(
        executor.entries[id.0].as_ref().unwrap().invocation,
        InvocationMode::OnNewData
    );

    // Change to Always
    executor.set_invocation(id, InvocationMode::Always);
    assert_eq!(
        executor.entries[id.0].as_ref().unwrap().invocation,
        InvocationMode::Always
    );
}

// ====================================================================
// Phase 49: ExecutorSemantics tests
// ====================================================================

#[test]
fn test_set_semantics() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // Default is RclcppExecutor
    assert_eq!(executor.semantics, ExecutorSemantics::RclcppExecutor);

    executor.set_semantics(ExecutorSemantics::LogicalExecutionTime);
    assert_eq!(executor.semantics, ExecutorSemantics::LogicalExecutionTime);
}

// ====================================================================
// Phase 47: LET semantics pre-sample behavior
// ====================================================================

#[test]
fn test_let_semantics_pre_samples_data() {
    // In LET mode, data is pre-sampled into the entry buffer before any
    // callback runs. This test verifies that the callback receives data
    // even though the mock subscriber's pending data is consumed during
    // the pre-sample phase (not during try_process).
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor.set_semantics(ExecutorSemantics::LogicalExecutionTime);
    let nid = executor
        .node_builder("test_let_semantics_pre_samples_data")
        .build()
        .unwrap();

    let received = std::sync::Arc::new(std::sync::Mutex::new(None));
    let received2 = received.clone();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", move |msg: &TestMsg| {
            *received2.lock().unwrap() = Some(msg.data);
        })
        .unwrap();

    // Load CDR data
    let (data, len) = encode_test_msg(77);
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe { &*(arena_ptr.add(meta.offset) as *const MockSubscriber) }.load(data, len);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 1);
    assert_eq!(*received.lock().unwrap(), Some(77));
}

#[test]
fn test_let_semantics_raw_subscription() {
    // Verify LET pre-sampling works for raw subscriptions too.
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor.set_semantics(ExecutorSemantics::LogicalExecutionTime);

    static RAW_LET_LEN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

    unsafe extern "C" fn raw_let_cb(_data: *const u8, len: usize, _ctx: *mut core::ffi::c_void) {
        RAW_LET_LEN.store(len, std::sync::atomic::Ordering::SeqCst);
    }

    RAW_LET_LEN.store(0, std::sync::atomic::Ordering::SeqCst);

    executor
        .add_arena_subscription_c_callback::<{ crate::config::DEFAULT_RX_BUF_SIZE }>(
            None,
            "/test",
            "test/msg/TestMsg",
            "test_hash",
            QoSProfile::default().keep_last(1),
            raw_let_cb,
            core::ptr::null_mut(),
            None, // no group
            // phase-402 W2 — no hint in this test; 0 = no opinion.
            0,
        )
        .unwrap();

    let (data, len) = encode_test_msg(42);
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe {
        let sub_ptr = arena_ptr.add(meta.offset) as *const MockSubscriber;
        (*sub_ptr).load(data, len);
    }

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 1);
    assert_eq!(RAW_LET_LEN.load(std::sync::atomic::Ordering::SeqCst), len);
}

// ====================================================================
// Phase 47: Trigger::All requires all non-timer handles
// ====================================================================

#[test]
fn test_trigger_all_with_mixed_handles() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // Add a timer and a subscription
    executor
        .register_timer(TimerDuration::from_millis(100), || {})
        .unwrap();
    let nid = executor
        .node_builder("test_trigger_all_with_mixed_handles")
        .build()
        .unwrap();
    let _sub_id = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_: &TestMsg| {})
        .unwrap();

    executor.set_trigger(Trigger::All);

    // Timer is always ready, but subscription has no data → trigger fails
    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(result.subscriptions_processed, 0);
    // Timer delta still accumulates

    // Now load data into subscription
    let (data, len) = encode_test_msg(1);
    let meta1 = executor.entries[1].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe { &*(arena_ptr.add(meta1.offset) as *const MockSubscriber) }.load(data, len);

    let result = elapse_then_spin_once(&mut executor, 100);
    assert_eq!(result.subscriptions_processed, 1);
    assert_eq!(result.timers_fired, 1);
}

// ====================================================================
// Phase 47: Trigger::AllOf sensor fusion pattern
// ====================================================================

#[test]
fn test_trigger_allof_fires_when_both_ready() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_trigger_allof_fires_when_both_ready")
        .build()
        .unwrap();
    let id_a = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/sensor_a", |_: &TestMsg| {})
        .unwrap();
    let id_b = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/sensor_b", |_: &TestMsg| {})
        .unwrap();

    // AllOf — dispatch only when BOTH subscriptions have data
    executor.set_trigger(Trigger::AllOf(id_a | id_b));

    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off_a = executor.entries[0].as_ref().unwrap().offset;
    let off_b = executor.entries[1].as_ref().unwrap().offset;

    // Load data only into sensor_a → trigger should NOT fire
    let (data, len) = encode_test_msg(1);
    unsafe { &*(arena_ptr.add(off_a) as *const MockSubscriber) }.load(data, len);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        result.subscriptions_processed, 0,
        "AllOf should not fire with only one ready"
    );

    // Now load data into both sensors → trigger should fire
    let (data_a, len_a) = encode_test_msg(10);
    let (data_b, len_b) = encode_test_msg(20);
    unsafe { &*(arena_ptr.add(off_a) as *const MockSubscriber) }.load(data_a, len_a);
    unsafe { &*(arena_ptr.add(off_b) as *const MockSubscriber) }.load(data_b, len_b);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        result.subscriptions_processed, 2,
        "AllOf should fire when both ready"
    );
}

#[test]
fn test_trigger_allof_empty_set_always_fires() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_trigger_allof_empty_set_always_fires")
        .build()
        .unwrap();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_: &TestMsg| {})
        .unwrap();

    // AllOf with empty set → vacuously true, always dispatches
    executor.set_trigger(Trigger::AllOf(HandleSet::EMPTY));

    // No data loaded, but trigger passes (empty set)
    let result = executor.spin_once(core::time::Duration::from_millis(0));
    // Subscription still has no data, so callback won't fire (take returns None)
    assert_eq!(result.subscriptions_processed, 0);
}

// ====================================================================
// Phase 47: Trigger::AnyOf dispatches on any handle in set
// ====================================================================

#[test]
fn test_trigger_anyof_fires_when_one_ready() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_trigger_anyof_fires_when_one_ready")
        .build()
        .unwrap();
    let id_a = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/topic_a", |_: &TestMsg| {})
        .unwrap();
    let id_b = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/topic_b", |_: &TestMsg| {})
        .unwrap();

    // AnyOf — dispatch when ANY handle in set has data
    executor.set_trigger(Trigger::AnyOf(id_a | id_b));

    // No data → trigger should NOT fire
    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        result.subscriptions_processed, 0,
        "AnyOf should not fire with none ready"
    );

    // Load data only into topic_a → trigger SHOULD fire
    let (data, len) = encode_test_msg(42);
    let meta_a = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe { &*(arena_ptr.add(meta_a.offset) as *const MockSubscriber) }.load(data, len);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(
        result.subscriptions_processed >= 1,
        "AnyOf should fire when one handle ready"
    );
}

#[test]
fn test_trigger_anyof_empty_set_never_fires() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("test_trigger_anyof_empty_set_never_fires")
        .build()
        .unwrap();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_: &TestMsg| {})
        .unwrap();

    // AnyOf with empty set → always false, never dispatches
    executor.set_trigger(Trigger::AnyOf(HandleSet::EMPTY));

    // Load data — trigger still won't pass (empty set, bits & 0 == 0)
    let (data, len) = encode_test_msg(1);
    let meta = executor.entries[0].as_ref().unwrap();
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    unsafe { &*(arena_ptr.add(meta.offset) as *const MockSubscriber) }.load(data, len);

    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        result.subscriptions_processed, 0,
        "AnyOf(EMPTY) should never fire"
    );
}

// ====================================================================
// Phase 49: Timer fires even when trigger fails
// ====================================================================

#[test]
fn test_timer_delta_accumulates_when_trigger_fails() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count2 = count.clone();

    executor
        .register_timer(TimerDuration::from_millis(100), move || {
            count2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();
    let nid = executor
        .node_builder("test_timer_delta_accumulates_when_trigger_fails")
        .build()
        .unwrap();
    let sub_id = executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/test", |_: &TestMsg| {})
        .unwrap();

    // Trigger requires specific handle that won't have data
    executor.set_trigger(Trigger::One(sub_id));

    // Timer delta accumulates even when trigger fails.
    // When the timer fires during the trigger-failed path, its callback
    // IS invoked (timers always fire regardless of trigger), but the
    // SpinOnceResult is not propagated.
    let _result = elapse_then_spin_once(&mut executor, 50); // elapsed=50, not ready
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 0);

    let _result = elapse_then_spin_once(&mut executor, 60); // elapsed=110, fires!
    // Timer callback fired even though trigger didn't pass
    assert_eq!(count.load(std::sync::atomic::Ordering::SeqCst), 1);
}

// ====================================================================
// Service type for Promise tests
// ====================================================================

/// Simple test service: AddTwoInts-like.
struct TestService;

#[derive(Debug, Clone, PartialEq)]
struct TestServiceRequest {
    a: i32,
}

#[derive(Debug, Clone, PartialEq)]
struct TestServiceReply {
    sum: i32,
}

impl RosMessage for TestServiceRequest {
    const TYPE_NAME: &'static str = "test/srv/TestService_Request";
    const TYPE_HASH: &'static str = "test_hash";
}

// phase-380 W4 — was `#[cfg(rmw_needs_type_descriptors)]`: the schema is
// now required by `MessageForRmw` on EVERY backend, because that is where
// a subscription's build-time size bound comes from.
impl nros_serdes::schema::Message for TestServiceRequest {
    const TYPE_NAME: &'static str = "test/srv/TestService_Request";
    const FIELDS: &'static [nros_serdes::schema::Field] = &[nros_serdes::schema::Field {
        name: "a",
        ty: nros_serdes::schema::FieldType::Int32,
        offset: 0,
    }];
}

impl Serialize for TestServiceRequest {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.a)
    }
}

impl Deserialize for TestServiceRequest {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            a: reader.read_i32()?,
        })
    }
}

impl RosMessage for TestServiceReply {
    const TYPE_NAME: &'static str = "test/srv/TestService_Reply";
    const TYPE_HASH: &'static str = "test_hash";
}

// phase-380 W4 — was `#[cfg(rmw_needs_type_descriptors)]`: the schema is
// now required by `MessageForRmw` on EVERY backend, because that is where
// a subscription's build-time size bound comes from.
impl nros_serdes::schema::Message for TestServiceReply {
    const TYPE_NAME: &'static str = "test/srv/TestService_Reply";
    const FIELDS: &'static [nros_serdes::schema::Field] = &[nros_serdes::schema::Field {
        name: "sum",
        ty: nros_serdes::schema::FieldType::Int32,
        offset: 0,
    }];
}

impl Serialize for TestServiceReply {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.sum)
    }
}

impl Deserialize for TestServiceReply {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            sum: reader.read_i32()?,
        })
    }
}

impl nros_core::RosService for TestService {
    type Request = TestServiceRequest;
    type Reply = TestServiceReply;
    const SERVICE_NAME: &'static str = "test/srv/dds_/TestService_";
    const SERVICE_HASH: &'static str = "test_hash";
}

#[test]
fn test_service_builder_qos() {
    // Phase 193.2 — NodeCtx service builder + convenient create_service.
    let mut exec: Executor = executor_with_clock(MockSession::new());
    let id = exec.node_builder("n").build().unwrap();

    // convenient (fork tier) — default services QoS
    let _h = exec
        .node_mut(id)
        .create_service::<TestService, _>("/svc", |req: &TestServiceRequest| TestServiceReply {
            sum: req.a,
        })
        .expect("convenient service builds");

    // builder (clone tier) — explicit QoS
    let _h2 = exec
        .node_mut(id)
        .service("/svc2")
        .qos(QoSProfile::default().reliable().keep_last(5))
        .build::<TestService, _>(|req: &TestServiceRequest| TestServiceReply { sum: req.a })
        .expect("service builder with qos builds");
}

#[test]
fn test_node_service_client_with_qos() {
    // Phase 193.2b — Node session-path create_service_with_qos /
    // create_client_with_qos (rclcpp-style qos overload).
    let mut executor: Executor = executor_with_clock(MockSession::new());
    let mut node = executor.create_node("n").unwrap();
    let q = QoSProfile::default().reliable().keep_last(7);
    let _srv = node
        .create_service_with_qos::<TestService>("/svc", q)
        .expect("service with qos");
    let _cli = node
        .create_client_with_qos::<TestService>("/svc", q)
        .expect("client with qos");
}

/// RFC-0041 / Phase 239.4 — a callback-based service client delivers the reply
/// to its typed closure at `spin_once` (no `Promise::take`). Drives the full
/// `call` → inject-reply → dispatch → callback path through the executor with the
/// mock transport.
#[test]
fn test_service_client_callback_fires_at_spin() {
    use crate::executor::arena::ServiceClientSendHeader;

    let mut executor: Executor = executor_with_clock(MockSession::new());
    let nid = executor.node_builder("svc_cb").build().unwrap();

    let got = std::sync::Arc::new(std::sync::Mutex::new(None));
    let got2 = got.clone();
    let mut client = executor
        .node_mut(nid)
        .create_client_with_callback::<TestService, _>("/svc", move |reply: &TestServiceReply| {
            *got2.lock().unwrap() = Some(reply.sum);
        })
        .unwrap();

    // Send a request → `pending = true` (the mock send is a no-op).
    client.call(&TestServiceRequest { a: 7 }).unwrap();
    assert_eq!(*got.lock().unwrap(), None, "no reply delivered yet");

    // Inject a reply into the arena entry's mock client. The send header is the
    // first field of the typed entry, so it sits at the entry offset.
    let off = executor.entries[0].as_ref().unwrap().offset;
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let hdr = unsafe {
        &*(arena_ptr.add(off)
            as *const ServiceClientSendHeader<{ crate::config::DEFAULT_RX_BUF_SIZE }>)
    };
    let mut buf = [0u8; 256];
    let mut writer = CdrWriter::new_with_header(&mut buf).unwrap();
    writer.write_i32(14).unwrap();
    let len = writer.position();
    let mut reply = [0u8; 256];
    reply[..len].copy_from_slice(&buf[..len]);
    hdr.handle.load_reply(reply, len);

    // Spin → dispatcher drains the reply, deserializes `TestServiceReply`, fires
    // the typed callback.
    let result = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(result.any_work(), "spin did work");
    assert_eq!(
        *got.lock().unwrap(),
        Some(14),
        "callback delivered the deserialized reply"
    );
}

/// RFC-0041 / Phase 239.4 — a callback-based action client delivers
/// goal-response / feedback / result to typed closures at `spin_once`. Drives
/// each receive by injecting into the entry's mock channels and spinning.
#[test]
fn test_action_client_callbacks_fire_at_spin() {
    use crate::executor::action_core::ActionClientCore;

    /// CDR-with-header encode of a single i32 (mirrors a `{ i32 }` message).
    /// RFC-0069 / issue 0418 — the action payload is the FIELDS, with no inner
    /// encapsulation header. This used to be `CdrWriter::new_with_header`,
    /// encoding the double-header convention 0418 retired: the enclosing
    /// message carries the only header, and the consumer splices it on.
    ///
    /// These tests kept passing after the producer was fixed because the
    /// consumer still sniffed for an inner header and found one HERE. They were
    /// asserting the old wire format against a new producer.
    fn encode_i32_cdr(v: i32) -> ([u8; 256], usize) {
        let mut b = [0u8; 256];
        let mut w = CdrWriter::new(&mut b);
        w.write_i32(v).unwrap();
        let n = w.position();
        let _ = w; // release the &mut b borrow so b can be returned (avoids clippy::drop_non_drop)
        (b, n)
    }
    fn buf256(src: &[u8]) -> ([u8; 256], usize) {
        let mut b = [0u8; 256];
        b[..src.len()].copy_from_slice(src);
        (b, src.len())
    }

    let mut executor: Executor = executor_with_clock(MockSession::new());
    let nid = executor.node_builder("act_cb").build().unwrap();

    let goal_resp = std::sync::Arc::new(std::sync::Mutex::new(None));
    let feedback = std::sync::Arc::new(std::sync::Mutex::new(None));
    let result = std::sync::Arc::new(std::sync::Mutex::new(None));
    let (gr2, fb2, rs2) = (goal_resp.clone(), feedback.clone(), result.clone());

    let mut client = executor
        .node_mut(nid)
        .create_action_client_with_callbacks::<TestAction, _, _, _>(
            "/act",
            move |id: &nros_core::GoalId, accepted: bool| {
                let mut c = [0u8; 8];
                c.copy_from_slice(&id.uuid[..8]);
                *gr2.lock().unwrap() = Some((u64::from_le_bytes(c), accepted));
            },
            move |_id: &nros_core::GoalId, fb: &TestFeedback| {
                *fb2.lock().unwrap() = Some(fb.progress);
            },
            move |_id: &nros_core::GoalId, st: nros_core::GoalStatus, r: &TestResult| {
                *rs2.lock().unwrap() = Some((st, r.value));
            },
        )
        .unwrap();

    // Reach the entry's core (first field of the entry → entry offset).
    let off = executor.entries[0].as_ref().unwrap().offset;
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let core = unsafe {
        &*(arena_ptr.add(off)
            as *const ActionClientCore<
                { crate::config::DEFAULT_RX_BUF_SIZE },
                { crate::config::DEFAULT_RX_BUF_SIZE },
                { crate::config::DEFAULT_RX_BUF_SIZE },
            >)
    };

    // send_goal → goal_counter = 1.
    let goal_id = client.send_goal(&TestGoal { order: 1 }).unwrap();

    // 1. Goal-response: header(4) + accepted(1). `try_recv_send_goal_reply`
    //    copies it into `result_buffer`; the dispatcher reads byte 4.
    let (gr, grl) = buf256(&[0, 1, 0, 0, 1]);
    core.send_goal_client.load_reply(gr, grl);
    executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        *goal_resp.lock().unwrap(),
        Some((1u64, true)),
        "goal-response"
    );

    // 2. Feedback: outer header(4) + GoalId(16) + inner CDR(header + i32).
    let mut fb = [0u8; 256];
    fb[0..4].copy_from_slice(&[0, 1, 0, 0]);
    fb[4..12].copy_from_slice(&1u64.to_le_bytes()); // GoalId uuid[..8] = counter 1
    let (inner, il) = encode_i32_cdr(7);
    fb[20..20 + il].copy_from_slice(&inner[..il]);
    core.feedback_subscriber.load(fb, 20 + il);
    executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(*feedback.lock().unwrap(), Some(7), "feedback deserialized");

    // 3. Result: header(4) + status@4 + pad(3) + inner CDR(header + i32) @8.
    client.get_result(&goal_id).unwrap();
    let mut rs = [0u8; 256];
    rs[0..4].copy_from_slice(&[0, 1, 0, 0]);
    rs[4] = 4; // GoalStatus::Succeeded
    let (inner_r, irl) = encode_i32_cdr(99);
    rs[8..8 + irl].copy_from_slice(&inner_r[..irl]);
    core.get_result_client.load_reply(rs, 8 + irl);
    executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        *result.lock().unwrap(),
        Some((nros_core::GoalStatus::Succeeded, 99)),
        "result deserialized"
    );
}

/// RFC-0041 / Phase 239.7 — two feedbacks arriving before a spin are **both**
/// delivered via the QoS-depth ring (vs the pre-239 single-buffer overwrite).
#[test]
fn test_action_client_feedback_burst_buffered() {
    use crate::executor::action_core::ActionClientCore;

    /// RFC-0069 / issue 0418 — the action payload is the FIELDS, with no inner
    /// encapsulation header. This used to be `CdrWriter::new_with_header`,
    /// encoding the double-header convention 0418 retired: the enclosing
    /// message carries the only header, and the consumer splices it on.
    ///
    /// These tests kept passing after the producer was fixed because the
    /// consumer still sniffed for an inner header and found one HERE. They were
    /// asserting the old wire format against a new producer.
    fn encode_i32_cdr(v: i32) -> ([u8; 256], usize) {
        let mut b = [0u8; 256];
        let mut w = CdrWriter::new(&mut b);
        w.write_i32(v).unwrap();
        let n = w.position();
        let _ = w; // release the &mut b borrow so b can be returned (avoids clippy::drop_non_drop)
        (b, n)
    }

    let mut executor: Executor = executor_with_clock(MockSession::new());
    let nid = executor.node_builder("act_burst").build().unwrap();

    let feedbacks = std::sync::Arc::new(std::sync::Mutex::new(std::vec::Vec::<i32>::new()));
    let fb2 = feedbacks.clone();
    let _client = executor
        .node_mut(nid)
        .create_action_client_with_callbacks::<TestAction, _, _, _>(
            "/act",
            |_: &nros_core::GoalId, _: bool| {},
            move |_: &nros_core::GoalId, fb: &TestFeedback| {
                fb2.lock().unwrap().push(fb.progress);
            },
            |_: &nros_core::GoalId, _: nros_core::GoalStatus, _: &TestResult| {},
        )
        .unwrap();

    let off = executor.entries[0].as_ref().unwrap().offset;
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let core = unsafe {
        &*(arena_ptr.add(off)
            as *const ActionClientCore<
                { crate::config::DEFAULT_RX_BUF_SIZE },
                { crate::config::DEFAULT_RX_BUF_SIZE },
                { crate::config::DEFAULT_RX_BUF_SIZE },
            >)
    };

    // Inject TWO feedbacks into the subscriber queue BEFORE spinning.
    for p in [7i32, 8i32] {
        let mut fb = [0u8; 256];
        fb[0..4].copy_from_slice(&[0, 1, 0, 0]);
        fb[4..12].copy_from_slice(&1u64.to_le_bytes());
        let (inner, il) = encode_i32_cdr(p);
        fb[20..20 + il].copy_from_slice(&inner[..il]);
        core.feedback_subscriber.load(fb, 20 + il);
    }

    // One spin drains BOTH into the ring and dispatches both — no overwrite.
    executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        *feedbacks.lock().unwrap(),
        std::vec![7, 8],
        "both burst feedbacks delivered via the QoS ring"
    );
}

/// Phase 189.M3.3.d — runtime proof that a **service** bound to a sched context
/// honours it in `spin_once`: two services bound to EDF contexts dispatch in
/// deadline order, not registration order. This is the runtime payoff of M3.3 —
/// services (now arena-registered + sched-bindable across C/C++) ride the same
/// SC-ordered dispatch as subscriptions (mirrors `test_edf_dispatch_order`).
#[test]
fn test_service_dispatch_respects_sched_context() {
    use crate::executor::sched_context::{DeadlinePolicy, OptUs, SchedClass, SchedContext};
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let firing_order = std::sync::Arc::new(std::sync::Mutex::new(std::vec::Vec::<i32>::new()));
    let order_late = firing_order.clone();
    let order_early = firing_order.clone();

    let nid = executor
        .node_builder("test_service_dispatch_respects_sched_context")
        .build()
        .unwrap();

    // Registered first (lower DescIdx → FIFO-first) → bind to the LATER deadline.
    let h_late = executor
        .node_mut(nid)
        .create_service::<TestService, _>("/late", move |req: &TestServiceRequest| {
            order_late.lock().unwrap().push(req.a);
            TestServiceReply { sum: req.a }
        })
        .unwrap();
    // Registered second → bind to the EARLIER deadline so EDF promotes it.
    let h_early = executor
        .node_mut(nid)
        .create_service::<TestService, _>("/early", move |req: &TestServiceRequest| {
            order_early.lock().unwrap().push(req.a);
            TestServiceReply { sum: req.a }
        })
        .unwrap();

    let sc_late = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Edf,
            deadline_us: OptUs::from_us(1000),
            deadline_policy: DeadlinePolicy::Activated,
            ..Default::default()
        })
        .unwrap();
    let sc_early = executor
        .create_sched_context(SchedContext {
            class: SchedClass::Edf,
            deadline_us: OptUs::from_us(100),
            deadline_policy: DeadlinePolicy::Activated,
            ..Default::default()
        })
        .unwrap();
    executor
        .bind_handle_to_sched_context(h_late, sc_late)
        .unwrap();
    executor
        .bind_handle_to_sched_context(h_early, sc_early)
        .unwrap();

    // Load a request into each mock server (req.a identifies which fired).
    let (d_late, n_late) = encode_test_msg(10);
    let (d_early, n_early) = encode_test_msg(20);
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off_late = executor.entries[0].as_ref().unwrap().offset;
    let off_early = executor.entries[1].as_ref().unwrap().offset;
    unsafe { &*(arena_ptr.add(off_late) as *const MockServiceServer) }.load(d_late, n_late);
    unsafe { &*(arena_ptr.add(off_early) as *const MockServiceServer) }.load(d_early, n_early);

    let _ = executor.spin_once(core::time::Duration::from_millis(0));

    let order = firing_order.lock().unwrap();
    // Earlier-deadline (req.a=20) must precede later-deadline (req.a=10).
    assert_eq!(*order, std::vec![20, 10]);
}

// ====================================================================
// Promise tests
// ====================================================================

#[test]
fn test_promise_try_recv_returns_none_then_some() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let mut node = executor.create_node("test").unwrap();
    let mut client = node.create_client::<TestService>("/test_svc").unwrap();

    // Start a non-blocking call
    let request = TestServiceRequest { a: 42 };
    let mut promise = client.call(&request).unwrap();

    // No reply loaded yet — should return None
    assert!(promise.take().unwrap().is_none());

    // Load a CDR-encoded reply into the mock
    let mut reply_buf = [0u8; 256];
    let mut writer = CdrWriter::new_with_header(&mut reply_buf).unwrap();
    writer.write_i32(99).unwrap();
    let reply_len = writer.position();

    // Access the mock client through the promise handle
    promise.handle.load_reply(reply_buf, reply_len);

    // Now take should return the reply
    let reply = promise.take().unwrap().unwrap();
    assert_eq!(reply.sum, 99);
}

// ====================================================================
// Wall-clock-accurate timer accumulation regression test (Phase 100
// follow-up: 232 Hz → 40 Hz fix).
// ====================================================================
//
// `spin_once(timeout)` used to credit the requested `timeout_ms` to the
// timer accumulator regardless of how long `drive_io` actually blocked.
// MockSession::drive_io returns immediately, so a 100 ms `spin_once`
// would tick a 30 ms timer ~3 times even though 0 wall-clock ms had
// elapsed. Under sustained traffic that broke a 30 Hz control loop into
// >200 Hz overshoot.
//
// The fix: measure real elapsed via `Instant::now()` and carry the
// sub-ms remainder across calls. This test asserts a 50 ms timer does
// NOT fire after a single 1 s `spin_once` against a no-op session.
#[test]
#[cfg(feature = "alloc")]
fn test_spin_once_does_not_credit_timeout_to_timer_delta() {
    use core::{
        sync::atomic::{AtomicU32, Ordering},
        time::Duration,
    };
    static FIRES: AtomicU32 = AtomicU32::new(0);
    FIRES.store(0, Ordering::SeqCst);

    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    // 50 ms periodic timer.
    let _timer = executor
        .register_timer(TimerDuration::from_millis(50), || {
            FIRES.fetch_add(1, Ordering::SeqCst);
        })
        .unwrap();

    // Ask for a 1 s spin. MockSession::drive_io returns instantly, so
    // real elapsed is ~0 ms. With the bug the timer would fire ~20 times
    // (1000 ms / 50 ms). Without the bug, 0 fires.
    let start = std::time::Instant::now();
    executor.spin_once(Duration::from_millis(1000));
    let real_elapsed_ms = start.elapsed().as_millis() as u64;

    let fires = FIRES.load(Ordering::SeqCst);

    // Expected fires = real_elapsed / 50 ms. Allow off-by-one for the
    // residual carry. Pre-fix this would be ~20 regardless of elapsed.
    let expected_max = (real_elapsed_ms / 50 + 1) as u32;
    assert!(
        fires <= expected_max,
        "timer over-fired: got {fires} fires after only {real_elapsed_ms} ms wall-clock \
         (expected ≤ {expected_max}). The pre-fix bug credited the requested \
         timeout (1000 ms) to the timer delta.",
    );
}

// ====================================================================
// Phase 172.K.5 — explicit per-node session-slot selection
// ====================================================================

/// `NodeBuilder::session_idx(n)` binds a Node directly to a pre-opened
/// session slot (the multi-domain routing primitive), bypassing rmw
/// resolution, and validates the slot against the opened set.
#[test]
fn node_builder_session_idx_binds_explicit_slot_and_validates() {
    let mut executor: Executor = executor_with_clock(MockSession::new());
    // Simulate `open_multi` having opened one extra session (slot 1).
    assert!(executor.extra_sessions.push(MockSession::new()).is_ok());

    // Slot 0 = primary.
    let n0 = executor.node_builder("n0").session_idx(0).build().unwrap();
    assert_eq!(executor.node(n0).unwrap().session_idx, 0);

    // Slot 1 = the extra session.
    let n1 = executor.node_builder("n1").session_idx(1).build().unwrap();
    assert_eq!(executor.node(n1).unwrap().session_idx, 1);

    // Out-of-range slot (only 0 + 1 exist) → error, not a silent bad bind.
    assert!(executor.node_builder("bad").session_idx(2).build().is_err());
}

// ============================================================================
// Phase 237 — deferred get_result (concurrent-safe seq routing)
// ============================================================================

/// Two concurrently-active goals each have a `get_result` request arrive while
/// they are still executing. The server must HOLD both replies (rclcpp_action
/// sends get_result right after acceptance) and, on completion, reply to each
/// using ITS OWN correlation token — never cross-wire them. This is the
/// backend-agnostic heart of Option A; the seq routing lives in
/// `ActionServerCore`, shared by the XRCE / Zenoh / Cyclone backends.
#[test]
fn test_get_result_deferred_per_goal_concurrent() {
    use super::action_core::{ActionServerCore, RawActiveGoal};
    use crate::mock::MockPublisher;
    use nros_core::{GoalId, GoalStatus};

    let mut core: ActionServerCore = ActionServerCore::from_channels(
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockPublisher,
        MockPublisher,
    );

    let g1 = GoalId { uuid: [1u8; 16] };
    let g2 = GoalId { uuid: [2u8; 16] };
    let _ = core.active_goals.push(RawActiveGoal {
        goal_id: g1,
        status: GoalStatus::Executing,
    });
    let _ = core.active_goals.push(RawActiveGoal {
        goal_id: g2,
        status: GoalStatus::Executing,
    });

    // A get_result request is [CDR-LE header][fixed uint8[16] goal_id].
    let mk_req = |g: &GoalId| -> ([u8; 256], usize) {
        let mut b = [0u8; 256];
        b[..4].copy_from_slice(&[0x00, 0x01, 0x00, 0x00]);
        b[4..20].copy_from_slice(&g.uuid);
        (b, 20)
    };
    let default_result = [0u8; 4];

    // g1's get_result arrives first (mock seq 0), then g2's (mock seq 1).
    let (r1, l1) = mk_req(&g1);
    core.get_result_server.load(r1, l1);
    core.try_handle_get_result_raw(&default_result).unwrap();
    let (r2, l2) = mk_req(&g2);
    core.get_result_server.load(r2, l2);
    core.try_handle_get_result_raw(&default_result).unwrap();

    // Both deferred; nothing replied while the goals are active.
    assert_eq!(core.pending_get_results.len(), 2);
    assert_eq!(core.get_result_server.sent.borrow().len(), 0);

    // Reply layout: [4-byte CDR header][i8 status][3 pad][result CDR] → result
    // bytes begin at offset 8.
    const RESULT_OFF: usize = 8;

    // Complete g2 FIRST (out of arrival order) → flush only g2's held request,
    // with g2's token (seq 1) — not g1's.
    let res2 = [0xBBu8; 8];
    core.complete_goal_raw(&g2, GoalStatus::Succeeded, &res2)
        .unwrap();
    {
        let sent = core.get_result_server.sent.borrow();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].0, 1, "g2 reply must use g2's correlation token");
        assert_eq!(sent[0].1[4], GoalStatus::Succeeded as u8 as i8 as u8);
        assert_eq!(&sent[0].1[RESULT_OFF..RESULT_OFF + res2.len()], &res2);
    }
    assert_eq!(core.pending_get_results.len(), 1);

    // Complete g1 → flush g1's held request with g1's token (seq 0).
    let res1 = [0xAAu8; 8];
    core.complete_goal_raw(&g1, GoalStatus::Succeeded, &res1)
        .unwrap();
    {
        let sent = core.get_result_server.sent.borrow();
        assert_eq!(sent.len(), 2);
        assert_eq!(sent[1].0, 0, "g1 reply must use g1's token, not g2's");
        assert_eq!(&sent[1].1[RESULT_OFF..RESULT_OFF + res1.len()], &res1);
    }
    assert_eq!(core.pending_get_results.len(), 0);
}

/// A `get_result` that arrives AFTER the goal already terminated is answered
/// immediately from the completed-results slab (the nano-ros↔nano-ros path),
/// never entering the pending table.
#[test]
fn test_get_result_after_completion_replies_immediately() {
    use super::action_core::ActionServerCore;
    use crate::mock::MockPublisher;
    use nros_core::{GoalId, GoalStatus};

    let mut core: ActionServerCore = ActionServerCore::from_channels(
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockPublisher,
        MockPublisher,
    );

    let g = GoalId { uuid: [7u8; 16] };
    // Goal completes before any get_result arrives.
    let res = [0xCCu8; 8];
    core.complete_goal_raw(&g, GoalStatus::Succeeded, &res)
        .unwrap();
    assert_eq!(core.pending_get_results.len(), 0);
    assert_eq!(core.get_result_server.sent.borrow().len(), 0);

    let mut b = [0u8; 256];
    b[..4].copy_from_slice(&[0x00, 0x01, 0x00, 0x00]);
    b[4..20].copy_from_slice(&g.uuid);
    core.get_result_server.load(b, 20);
    core.try_handle_get_result_raw(&[0u8; 4]).unwrap();

    // Replied immediately, not deferred.
    assert_eq!(core.pending_get_results.len(), 0);
    let sent = core.get_result_server.sent.borrow();
    assert_eq!(sent.len(), 1);
    assert_eq!(&sent[0].1[8..8 + res.len()], &res);
}

// ============================================================================
// Issue 0796 — the completed-result slab must RECLAIM
// ============================================================================

/// A `get_result` request wire frame: `[CDR-LE header][fixed uint8[16] goal_id]`.
#[cfg(test)]
fn mk_get_result_req(goal_id: &nros_core::GoalId) -> ([u8; 256], usize) {
    let mut b = [0u8; 256];
    b[..4].copy_from_slice(&[0x00, 0x01, 0x00, 0x00]);
    b[4..20].copy_from_slice(&goal_id.uuid);
    (b, 20)
}

/// Reply layout is `[4-byte CDR header][i8 status][3 pad][result CDR]`, so the
/// status byte is at 4 and the result bytes start at 8.
#[cfg(test)]
const REPLY_STATUS_OFF: usize = 4;
#[cfg(test)]
const REPLY_RESULT_OFF: usize = 8;

/// Fetch `goal_id`'s result and return `(status_byte, result_bytes)` from the
/// single reply the mock recorded. Clears the mock's reply log, which holds
/// only 8 entries and DROPS the overflow — a loop that trusted `sent.len()`
/// would stop seeing replies rather than fail.
#[cfg(test)]
fn fetch_result<const G: usize, const R: usize, const F: usize, const M: usize>(
    core: &mut super::action_core::ActionServerCore<G, R, F, M>,
    goal_id: &nros_core::GoalId,
    default_result: &[u8],
    want_len: usize,
) -> (u8, heapless::Vec<u8, 256>) {
    let (req, len) = mk_get_result_req(goal_id);
    core.get_result_server.load(req, len);
    core.try_handle_get_result_raw(default_result).unwrap();
    let (status, bytes) = {
        let sent = core.get_result_server.sent.borrow();
        assert_eq!(sent.len(), 1, "expected exactly one reply for this fetch");
        let mut out: heapless::Vec<u8, 256> = heapless::Vec::new();
        out.extend_from_slice(&sent[0].1[REPLY_RESULT_OFF..REPLY_RESULT_OFF + want_len])
            .unwrap();
        (sent[0].1[REPLY_STATUS_OFF], out)
    };
    core.get_result_server.sent.borrow_mut().clear();
    (status, bytes)
}

/// Issue 0796, problem 1 — completing goals whose results TOTAL more than
/// `RESULT_BUF` must keep delivering.
///
/// The pre-fix slab was a bump allocator with no free: `result_slab_used` only
/// ever moved forward, so once the accumulated results crossed `RESULT_BUF`
/// every later completion was dropped on the floor (`stored = false`, no entry
/// pushed, `complete_goal_raw` returning `()`), and every subsequent
/// `get_result` was answered `UNKNOWN` with the default payload. Here
/// `RESULT_BUF` is 128 and each result is 40 bytes, so the third completion
/// fills it and the fourth is where the old code starts lying.
#[test]
fn action_results_keep_being_delivered_past_the_slab_capacity() {
    use super::action_core::ActionServerCore;
    use crate::mock::MockPublisher;
    use nros_core::{GoalId, GoalStatus};

    // GOAL_BUF 256, RESULT_BUF 128, FEEDBACK_BUF 128, MAX_GOALS 4.
    let mut core: ActionServerCore<256, 128, 128, 4> = ActionServerCore::from_channels(
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockPublisher,
        MockPublisher,
    );

    const RESULT_LEN: usize = 40;
    // 10 goals x 40 bytes = 400 bytes through a 128-byte slab.
    const GOALS: u8 = 10;
    let default_result = [0u8; 4];

    for i in 1..=GOALS {
        let g = GoalId { uuid: [i; 16] };
        let res = [0xA0u8.wrapping_add(i); RESULT_LEN];

        core.complete_goal_raw(&g, GoalStatus::Succeeded, &res)
            .unwrap_or_else(|e| panic!("goal {i} could not be stored: {e:?}"));

        let (status, bytes) = fetch_result(&mut core, &g, &default_result, RESULT_LEN);
        assert_eq!(
            status,
            GoalStatus::Succeeded as u8 as i8 as u8,
            "goal {i} was answered with a non-terminal status - the slab dropped it"
        );
        assert_eq!(
            &bytes[..],
            &res[..],
            "goal {i} was answered with the wrong result bytes"
        );

        // The reclamation is real, not just "it still answers": live bytes
        // never exceed the slab, and the entry table never exceeds MAX_GOALS.
        assert!(core.result_slab_used() <= 128);
        assert!(core.completed_result_count() <= 4);
    }
}

/// Same overflow, through the DEFERRED path: the client sends `get_result`
/// while the goal is still active (what `rclcpp_action` does), so the reply is
/// flushed by `complete_goal_raw`.
///
/// Pre-fix this was the worse half of the bug — the flush was inside
/// `if stored`, so a client that had already asked was never answered at all
/// and waited on its result future forever.
#[test]
fn deferred_get_result_is_answered_past_the_slab_capacity() {
    use super::action_core::{ActionServerCore, RawActiveGoal};
    use crate::mock::MockPublisher;
    use nros_core::{GoalId, GoalStatus};

    let mut core: ActionServerCore<256, 128, 128, 4> = ActionServerCore::from_channels(
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockPublisher,
        MockPublisher,
    );

    const RESULT_LEN: usize = 40;
    let default_result = [0u8; 4];

    for i in 1..=10u8 {
        let g = GoalId { uuid: [i; 16] };
        let res = [0x50u8.wrapping_add(i); RESULT_LEN];

        core.active_goals
            .push(RawActiveGoal {
                goal_id: g,
                status: GoalStatus::Executing,
            })
            .ok()
            .unwrap();

        // Client asks while the goal is still running -> deferred.
        let (req, len) = mk_get_result_req(&g);
        core.get_result_server.load(req, len);
        core.try_handle_get_result_raw(&default_result).unwrap();
        assert_eq!(core.pending_get_results.len(), 1, "goal {i} not deferred");
        assert_eq!(core.get_result_server.sent.borrow().len(), 0);

        core.complete_goal_raw(&g, GoalStatus::Succeeded, &res)
            .unwrap_or_else(|e| panic!("goal {i} could not be stored: {e:?}"));

        assert_eq!(
            core.pending_get_results.len(),
            0,
            "goal {i}: the held get_result was never flushed"
        );
        {
            let sent = core.get_result_server.sent.borrow();
            assert_eq!(sent.len(), 1, "goal {i}: no reply reached the waiter");
            assert_eq!(
                &sent[0].1[REPLY_RESULT_OFF..REPLY_RESULT_OFF + RESULT_LEN],
                &res[..],
                "goal {i}: waiter got the wrong result bytes"
            );
        }
        core.get_result_server.sent.borrow_mut().clear();
    }
}

/// A completed result survives until the slab is actually under pressure, and
/// when something must go the FETCHED result goes first.
///
/// Issue 0796's eviction contract: a delivered result does not pin storage, and
/// a client that never asks cannot wedge the server.
#[test]
fn completed_result_survives_until_pressure_then_evicts_the_fetched_one_first() {
    use super::action_core::ActionServerCore;
    use crate::mock::MockPublisher;
    use nros_core::{GoalId, GoalStatus};

    // 128-byte slab: exactly three 40-byte results fit.
    let mut core: ActionServerCore<256, 128, 128, 4> = ActionServerCore::from_channels(
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockPublisher,
        MockPublisher,
    );

    const RESULT_LEN: usize = 40;
    let default_result = [0u8; 4];
    let g = |i: u8| GoalId { uuid: [i; 16] };
    let payload = |i: u8| [i; RESULT_LEN];

    for i in 1..=3u8 {
        core.complete_goal_raw(&g(i), GoalStatus::Succeeded, &payload(i))
            .unwrap();
    }
    assert_eq!(core.completed_result_count(), 3);
    assert_eq!(core.result_slab_used(), 120);

    // Nothing has been evicted yet: a client that asks AFTER completion still
    // gets its result, including the oldest one.
    let (status, bytes) = fetch_result(&mut core, &g(2), &default_result, RESULT_LEN);
    assert_eq!(status, GoalStatus::Succeeded as u8 as i8 as u8);
    assert_eq!(&bytes[..], &payload(2)[..]);

    // A fourth completion needs 40 bytes and only 8 are free -> reclaim. g2 was
    // fetched, so g2 is what goes, NOT the older-but-unfetched g1.
    core.complete_goal_raw(&g(4), GoalStatus::Succeeded, &payload(4))
        .unwrap();

    assert!(core.has_completed_result(&g(1)), "unfetched g1 was evicted");
    assert!(core.has_completed_result(&g(3)), "unfetched g3 was evicted");
    assert!(core.has_completed_result(&g(4)));
    assert!(
        !core.has_completed_result(&g(2)),
        "the already-delivered g2 should have been reclaimed first"
    );

    // Compaction rewrote the survivors' offsets correctly - each still reads back
    // as its own payload.
    for i in [1u8, 3, 4] {
        let (status, bytes) = fetch_result(&mut core, &g(i), &default_result, RESULT_LEN);
        assert_eq!(status, GoalStatus::Succeeded as u8 as i8 as u8, "g{i}");
        assert_eq!(&bytes[..], &payload(i)[..], "g{i} read back corrupted");
    }

    // The evicted goal is answered honestly (UNKNOWN + default), never left
    // hanging.
    let (status, _) = fetch_result(&mut core, &g(2), &default_result, default_result.len());
    assert_eq!(status, GoalStatus::Unknown as u8 as i8 as u8);
}

/// A result larger than `RESULT_BUF` can never be retained — that is the one
/// remaining failure, and `complete_goal_raw` now REPORTS it (issue 0796: it
/// returned `()`).  The waiting requester is still answered, from the caller's
/// own bytes.
#[test]
fn an_unretainable_result_is_reported_and_the_waiter_is_still_answered() {
    use super::{
        action_core::{ActionServerCore, RawActiveGoal},
        types::NodeError,
    };
    use crate::mock::MockPublisher;
    use nros_core::{GoalId, GoalStatus};

    let mut core: ActionServerCore<256, 128, 128, 4> = ActionServerCore::from_channels(
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockPublisher,
        MockPublisher,
    );

    let g = GoalId { uuid: [9u8; 16] };
    let res = [0x77u8; 200]; // > RESULT_BUF (128)
    core.active_goals
        .push(RawActiveGoal {
            goal_id: g,
            status: GoalStatus::Executing,
        })
        .ok()
        .unwrap();

    let (req, len) = mk_get_result_req(&g);
    core.get_result_server.load(req, len);
    core.try_handle_get_result_raw(&[0u8; 4]).unwrap();
    assert_eq!(core.pending_get_results.len(), 1);

    let err = core
        .complete_goal_raw(&g, GoalStatus::Succeeded, &res)
        .expect_err("a result larger than RESULT_BUF must not report success");
    assert!(matches!(err, NodeError::BufferTooSmall));

    // ... and the client that was already waiting is not stranded.
    assert_eq!(core.pending_get_results.len(), 0);
    let sent = core.get_result_server.sent.borrow();
    assert_eq!(sent.len(), 1);
    assert_eq!(
        &sent[0].1[REPLY_RESULT_OFF..REPLY_RESULT_OFF + res.len()],
        &res[..]
    );
}

/// `expire_completed_results` is the eager analogue of
/// `rcl_action_expire_goals()`: it reclaims exactly the delivered results and
/// leaves the rest.
#[test]
fn expire_completed_results_reclaims_only_delivered_results() {
    use super::action_core::ActionServerCore;
    use crate::mock::MockPublisher;
    use nros_core::{GoalId, GoalStatus};

    let mut core: ActionServerCore<256, 128, 128, 4> = ActionServerCore::from_channels(
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockPublisher,
        MockPublisher,
    );

    const RESULT_LEN: usize = 40;
    let g = |i: u8| GoalId { uuid: [i; 16] };
    let payload = |i: u8| [i; RESULT_LEN];
    for i in 1..=3u8 {
        core.complete_goal_raw(&g(i), GoalStatus::Succeeded, &payload(i))
            .unwrap();
    }

    // Nothing fetched yet -> nothing to expire.
    assert_eq!(core.expire_completed_results(), 0);
    assert_eq!(core.result_slab_used(), 120);

    let _ = fetch_result(&mut core, &g(1), &[0u8; 4], RESULT_LEN);
    let _ = fetch_result(&mut core, &g(3), &[0u8; 4], RESULT_LEN);

    assert_eq!(core.expire_completed_results(), 2);
    assert_eq!(core.completed_result_count(), 1);
    assert_eq!(core.result_slab_used(), RESULT_LEN);
    assert!(core.has_completed_result(&g(2)));

    // The survivor was compacted down to offset 0 and still reads back intact.
    let (_, bytes) = fetch_result(&mut core, &g(2), &[0u8; 4], RESULT_LEN);
    assert_eq!(&bytes[..], &payload(2)[..]);
}

// ============================================================================
// Phase 272 (RFC-0047) — node_name → sched-context table + node_builder lookup
// ============================================================================

/// Seeded name resolves to the bound SC, and a callback registered under
/// that node inherits SC 2 via `apply_node_default_sched`.
#[test]
fn test_bind_node_name_sched_seeded_resolves() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    // SC slots 1 and 2 must exist for apply_node_default_sched to wire the
    // binding. create_sched_context fills slots in order starting at 1.
    let _sc1 = executor
        .create_sched_context(SchedContext::default())
        .unwrap();
    let sc2 = executor
        .create_sched_context(SchedContext::default())
        .unwrap();
    assert_eq!(sc2, SchedContextId(2));

    // Seed the table before building.
    executor.bind_node_name_sched("talker", "/", SchedContextId(2));

    let nid = executor.node_builder("talker").build().unwrap();

    // 1. The NodeRecord itself carries the seeded SC.
    assert_eq!(executor.nodes[nid.index()].default_sched, SchedContextId(2));

    // 2. A callback registered under the node inherits SC 2 via
    //    apply_node_default_sched (called inside create_subscription).
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/t", |_: &TestMsg| {})
        .unwrap();
    assert_eq!(executor.sched_context_bindings[0], SchedContextId(2));
}

/// An unseeded name always produces SchedContextId(0) (default path unchanged).
#[test]
fn test_bind_node_name_sched_unseeded_defaults_to_zero() {
    use crate::executor::sched_context::SchedContextId;

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    // No bind call — table is empty.
    let nid = executor.node_builder("listener").build().unwrap();
    assert_eq!(executor.nodes[nid.index()].default_sched, SchedContextId(0));
}

/// An explicit `.sched(id)` on the builder wins over a conflicting table entry.
#[test]
fn test_bind_node_name_sched_explicit_beats_table() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(1)
    executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(2)

    // Table says SC 2, but explicit .sched(1) should win.
    executor.bind_node_name_sched("talker", "/", SchedContextId(2));

    let nid = executor
        .node_builder("talker")
        .sched(SchedContextId(1))
        .build()
        .unwrap();

    assert_eq!(
        executor.nodes[nid.index()].default_sched,
        SchedContextId(1),
        "explicit .sched() must override the table entry"
    );
}

/// A namespace-qualified key disambiguates nodes with the same name in
/// different namespaces — seeding "/ns"/"talker" does not affect "/"/"talker".
#[test]
fn test_bind_node_name_sched_namespace_disambiguates() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(1)

    // Seed for the namespaced node only.
    executor.bind_node_name_sched("talker", "/ns", SchedContextId(1));

    // Node in "/ns" namespace → gets SC 1.
    let nid_ns = executor
        .node_builder("talker")
        .namespace("/ns")
        .build()
        .unwrap();
    assert_eq!(
        executor.nodes[nid_ns.index()].default_sched,
        SchedContextId(1)
    );

    // Node in root "/" namespace (executor default) → not seeded → SC 0.
    let nid_root = executor.node_builder("talker").build().unwrap();
    assert_eq!(
        executor.nodes[nid_root.index()].default_sched,
        SchedContextId(0),
        "root-ns node must not inherit the /ns-keyed table entry"
    );
}

// =========================================================================
// Phase 273 (RFC-0047) — group_sched_table tests
// =========================================================================

/// Seeding a group binding causes a callback created under that group to
/// bind to the group's SC, not the node default (SC 0).
#[test]
fn test_bind_group_sched_seeded_resolves() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    // Create SC slots: 1 and 2 must exist so the validity checks pass.
    let _sc1 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(1)
    let sc2 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(2)
    assert_eq!(sc2, SchedContextId(2));

    // Seed the group table: node "node" / group "ctrl" → SC 2.
    executor.bind_group_sched("node", "/", "ctrl", SchedContextId(2));

    // Build the node (no default_sched seeded — node stays at SC 0).
    let nid = executor.node_builder("node").build().unwrap();
    assert_eq!(
        executor.nodes[nid.index()].default_sched,
        SchedContextId(0),
        "node default must remain 0 — group table is the override"
    );

    // Manually call apply_node_default_sched with group = Some("ctrl").
    // slot 0 is unused here; use it directly.
    executor.apply_node_default_sched(0, Some(nid), Some("ctrl"));
    assert_eq!(
        executor.sched_context_bindings[0],
        SchedContextId(2),
        "group table entry must win over node default (SC 0)"
    );
}

/// Two groups on the SAME node each resolve to their own SC (sub-node split).
#[test]
fn test_bind_group_sched_sub_node_split() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    let _sc1 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(1)
    let sc2 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(2)
    let sc3 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(3)
    assert_eq!(sc2, SchedContextId(2));
    assert_eq!(sc3, SchedContextId(3));

    // One node, two groups, two SCs — the capability the node-name table can't express.
    executor.bind_group_sched("node", "/", "ctrl", SchedContextId(2));
    executor.bind_group_sched("node", "/", "telem", SchedContextId(3));

    let nid = executor.node_builder("node").build().unwrap();

    // ctrl group callback → SC 2.
    executor.apply_node_default_sched(0, Some(nid), Some("ctrl"));
    assert_eq!(executor.sched_context_bindings[0], SchedContextId(2));

    // telem group callback → SC 3.
    executor.apply_node_default_sched(1, Some(nid), Some("telem"));
    assert_eq!(executor.sched_context_bindings[1], SchedContextId(3));
}

/// No group passed (None) → falls through to node default (phase-272 behaviour).
#[test]
fn test_bind_group_sched_no_group_uses_node_default() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    let _sc1 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(1)
    let sc2 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(2)
    assert_eq!(sc2, SchedContextId(2));

    // Seed both the node-name table and the group table.
    executor.bind_node_name_sched("node", "/", SchedContextId(2));
    executor.bind_group_sched("node", "/", "ctrl", SchedContextId(1));

    let nid = executor.node_builder("node").build().unwrap();
    assert_eq!(executor.nodes[nid.index()].default_sched, SchedContextId(2));

    // group = None → node default (SC 2), not the group entry.
    executor.apply_node_default_sched(0, Some(nid), None);
    assert_eq!(executor.sched_context_bindings[0], SchedContextId(2));
}

/// An unmapped group name falls back to the node default.
#[test]
fn test_bind_group_sched_unmapped_group_falls_back_to_node_default() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    let _sc1 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(1)
    let sc2 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(2)
    assert_eq!(sc2, SchedContextId(2));

    // Node default = SC 2; no group entry for "unknown_group".
    executor.bind_node_name_sched("node", "/", SchedContextId(2));

    let nid = executor.node_builder("node").build().unwrap();
    assert_eq!(executor.nodes[nid.index()].default_sched, SchedContextId(2));

    // Passing an unmapped group → falls back to node default (SC 2).
    executor.apply_node_default_sched(0, Some(nid), Some("unknown_group"));
    assert_eq!(
        executor.sched_context_bindings[0],
        SchedContextId(2),
        "unmapped group must fall back to node default"
    );
}

/// Group table entry wins over node default when both are seeded (precedence).
#[test]
fn test_bind_group_sched_group_beats_node_default() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    // SC 1, 2, 5 (skipping 3 and 4 to test non-contiguous ids).
    let _sc1 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(1)
    let _sc2 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(2)
    let _sc3 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(3)
    let _sc4 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(4)
    let sc5 = executor
        .create_sched_context(SchedContext::default())
        .unwrap(); // SchedContextId(5)
    let sc2 = SchedContextId(2);
    assert_eq!(sc5, SchedContextId(5));

    // node default = SC 5; "ctrl" group → SC 2.
    executor.bind_node_name_sched("node", "/", SchedContextId(5));
    executor.bind_group_sched("node", "/", "ctrl", sc2);

    let nid = executor.node_builder("node").build().unwrap();
    assert_eq!(executor.nodes[nid.index()].default_sched, SchedContextId(5));

    // group "ctrl" → SC 2 (group beats node default).
    executor.apply_node_default_sched(0, Some(nid), Some("ctrl"));
    assert_eq!(
        executor.sched_context_bindings[0],
        SchedContextId(2),
        "group table entry must beat node default"
    );
}

// Phase 273 W3 — high-level CallbackGroup API tests
// =========================================================================
// These tests exercise the user-facing create_callback_group / create_timer_in /
// create_subscription_in API end-to-end through the NodeCtx, confirming that
// the group name is threaded through to apply_node_default_sched and the
// executor's sched_context_bindings reflect the group's SC (not the node
// default or SC 0).

/// Timer created **in** a named group binds to the group's SC, not SC 0.
#[test]
fn test_callback_group_timer_in_group_binds_to_group_sc() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    // Create SC 1 and SC 2.
    let _sc1 = executor
        .create_sched_context(SchedContext::default())
        .unwrap();
    let sc2 = executor
        .create_sched_context(SchedContext::default())
        .unwrap();
    assert_eq!(sc2, SchedContextId(2));

    // Seed: group "ctrl" on node "node" → SC 2.
    executor.bind_group_sched("node", "/", "ctrl", SchedContextId(2));

    let nid = executor.node_builder("node").build().unwrap();
    // Node default is SC 0 (no bind_node_name_sched).
    assert_eq!(executor.nodes[nid.index()].default_sched, SchedContextId(0));

    // create_callback_group returns a value type — no borrow conflict.
    let group = executor.node_mut(nid).create_callback_group("ctrl");
    // create_timer_in threads the group name through register_timer_on.
    executor
        .node_mut(nid)
        .create_timer_in(&group, TimerDuration::from_millis(100), || {})
        .expect("create_timer_in");

    // The timer occupies slot 0; its SC binding must be the group's SC 2.
    assert_eq!(
        executor.sched_context_bindings[0],
        SchedContextId(2),
        "timer created in 'ctrl' group must bind to SC 2 (not node default SC 0)"
    );
}

/// Subscription created **in** a named group binds to the group's SC.
#[test]
fn test_callback_group_subscription_in_group_binds_to_group_sc() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    let _sc1 = executor
        .create_sched_context(SchedContext::default())
        .unwrap();
    let sc2 = executor
        .create_sched_context(SchedContext::default())
        .unwrap();
    assert_eq!(sc2, SchedContextId(2));

    // Seed: group "telem" on node "sensor" → SC 2.
    executor.bind_group_sched("sensor", "/", "telem", SchedContextId(2));

    let nid = executor.node_builder("sensor").build().unwrap();

    let group = executor.node_mut(nid).create_callback_group("telem");
    executor
        .node_mut(nid)
        .create_subscription_in::<TestMsg, _>(&group, "/sensor/data", |_: &TestMsg| {})
        .expect("create_subscription_in");

    // Subscription occupies slot 0; SC must be the group's SC 2.
    assert_eq!(
        executor.sched_context_bindings[0],
        SchedContextId(2),
        "subscription created in 'telem' group must bind to SC 2"
    );
}

/// create_callback_group with no matching group_sched_table entry falls back
/// to the node default SC (same behavior as passing no group at all).
#[test]
fn test_callback_group_unmapped_group_falls_back_to_node_default() {
    use crate::executor::sched_context::{SchedContext, SchedContextId};

    let session = MockSession::new();
    let mut executor = executor_with_clock(session);

    let _sc1 = executor
        .create_sched_context(SchedContext::default())
        .unwrap();
    let sc2 = executor
        .create_sched_context(SchedContext::default())
        .unwrap();
    assert_eq!(sc2, SchedContextId(2));

    // node "ctrl_node" has a default SC of 2; the group "unknown" has no entry.
    executor.bind_node_name_sched("ctrl_node", "/", SchedContextId(2));

    let nid = executor.node_builder("ctrl_node").build().unwrap();
    assert_eq!(executor.nodes[nid.index()].default_sched, SchedContextId(2));

    let group = executor.node_mut(nid).create_callback_group("unknown");
    executor
        .node_mut(nid)
        .create_timer_in(&group, TimerDuration::from_millis(50), || {})
        .expect("create_timer_in");

    // Falls back to node default (SC 2), not SC 0.
    assert_eq!(
        executor.sched_context_bindings[0],
        SchedContextId(2),
        "unmapped group must fall back to node default SC 2"
    );
}

// ====================================================================
// RFC-0052 / phase-296 W3b.5 — deadline-miss actions
// ====================================================================

/// `DeadlineAction::Skip`: a callback overrunning its SC deadline masks
/// the SAME SC's remaining callbacks for the rest of that spin cycle,
/// and the miss lands on the violation drain.
#[test]
fn deadline_skip_masks_remaining_same_sc_callbacks() {
    use crate::executor::sched_context::{DeadlineAction, OptUs, SchedContext};
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let fired = std::sync::Arc::new(std::sync::Mutex::new(std::vec::Vec::<i32>::new()));
    let f_slow = fired.clone();
    let f_victim = fired.clone();

    let nid = executor.node_builder("deadline_skip").build().unwrap();
    let h_slow = executor
        .node_mut(nid)
        .create_service::<TestService, _>("/slow", move |req: &TestServiceRequest| {
            f_slow.lock().unwrap().push(req.a);
            std::thread::sleep(core::time::Duration::from_millis(5));
            TestServiceReply { sum: req.a }
        })
        .unwrap();
    let h_victim = executor
        .node_mut(nid)
        .create_service::<TestService, _>("/victim", move |req: &TestServiceRequest| {
            f_victim.lock().unwrap().push(req.a);
            TestServiceReply { sum: req.a }
        })
        .unwrap();

    // One SC, 1 ms deadline, skip on miss — both callbacks bound to it.
    let sc = executor
        .create_sched_context(SchedContext {
            deadline_us: OptUs::from_us(1_000),
            deadline_action: DeadlineAction::Skip,
            ..Default::default()
        })
        .unwrap();
    executor.bind_handle_to_sched_context(h_slow, sc).unwrap();
    executor.bind_handle_to_sched_context(h_victim, sc).unwrap();

    let (d_slow, n_slow) = encode_test_msg(1);
    let (d_victim, n_victim) = encode_test_msg(2);
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off_slow = executor.entries[0].as_ref().unwrap().offset;
    let off_victim = executor.entries[1].as_ref().unwrap().offset;
    unsafe { &*(arena_ptr.add(off_slow) as *const MockServiceServer) }.load(d_slow, n_slow);
    unsafe { &*(arena_ptr.add(off_victim) as *const MockServiceServer) }.load(d_victim, n_victim);

    let _ = executor.spin_once(core::time::Duration::from_millis(0));

    // The slow callback ran 5 ms past a 1 ms deadline; its SC sibling
    // must NOT have fired in the same cycle.
    assert_eq!(*fired.lock().unwrap(), std::vec![1], "victim skipped");
    let mut rules = std::vec::Vec::new();
    executor.drain_violations(|v| rules.push((v.rule, v.declared)));
    assert!(
        rules
            .iter()
            .any(|(r, d)| *r == "deadline-miss-runtime" && *d == 1_000),
        "miss reported: {rules:?}"
    );

    // Next cycle: the mask is per-cycle — the victim fires now.
    let (d2, n2) = encode_test_msg(3);
    unsafe { &*(arena_ptr.add(off_victim) as *const MockServiceServer) }.load(d2, n2);
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert_eq!(
        *fired.lock().unwrap(),
        std::vec![1, 3],
        "mask cleared next cycle"
    );
}

/// `DeadlineAction::Warn`: reports the miss but never masks siblings.
#[test]
fn deadline_warn_reports_without_skipping() {
    use crate::executor::sched_context::{DeadlineAction, OptUs, SchedContext};
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let fired = std::sync::Arc::new(std::sync::Mutex::new(std::vec::Vec::<i32>::new()));
    let f_slow = fired.clone();
    let f_peer = fired.clone();

    let nid = executor.node_builder("deadline_warn").build().unwrap();
    let h_slow = executor
        .node_mut(nid)
        .create_service::<TestService, _>("/slow", move |req: &TestServiceRequest| {
            f_slow.lock().unwrap().push(req.a);
            std::thread::sleep(core::time::Duration::from_millis(5));
            TestServiceReply { sum: req.a }
        })
        .unwrap();
    let h_peer = executor
        .node_mut(nid)
        .create_service::<TestService, _>("/peer", move |req: &TestServiceRequest| {
            f_peer.lock().unwrap().push(req.a);
            TestServiceReply { sum: req.a }
        })
        .unwrap();
    let sc = executor
        .create_sched_context(SchedContext {
            deadline_us: OptUs::from_us(1_000),
            deadline_action: DeadlineAction::Warn,
            ..Default::default()
        })
        .unwrap();
    executor.bind_handle_to_sched_context(h_slow, sc).unwrap();
    executor.bind_handle_to_sched_context(h_peer, sc).unwrap();

    let (d1, n1) = encode_test_msg(1);
    let (d2, n2) = encode_test_msg(2);
    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off1 = executor.entries[0].as_ref().unwrap().offset;
    let off2 = executor.entries[1].as_ref().unwrap().offset;
    unsafe { &*(arena_ptr.add(off1) as *const MockServiceServer) }.load(d1, n1);
    unsafe { &*(arena_ptr.add(off2) as *const MockServiceServer) }.load(d2, n2);

    let _ = executor.spin_once(core::time::Duration::from_millis(0));

    assert_eq!(*fired.lock().unwrap(), std::vec![1, 2], "peer still fires");
    let mut misses = 0;
    executor.drain_violations(|v| {
        if v.rule == "deadline-miss-runtime" {
            misses += 1;
        }
    });
    assert!(misses >= 1, "warn reported the miss");
}

/// Issue #505 — a stalled tier must SAY it dropped activations.
///
/// The rate rule cannot: `check_rate` samples publish counts over a ~5 s
/// window, so an isolated stall is a fraction of a percent, and under
/// `CatchUp` the replayed activations refill the window entirely. The
/// overrun counter is exact and immediate, and `run_contract_monitors`
/// turns its delta into a `timer-overrun-runtime` violation.
#[cfg(feature = "alloc")]
#[test]
fn a_stalled_timer_reports_a_timer_overrun_violation() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    let fires = std::sync::Arc::new(std::sync::Mutex::new(0u32));
    let f = fires.clone();
    let id = executor
        .register_timer(TimerDuration::from_millis(10), move || {
            *f.lock().unwrap() += 1;
        })
        .unwrap();

    // A stall worth ~12 periods: one activation, the rest dropped.
    let _ = elapse_then_spin_once(&mut executor, 120);
    assert_eq!(*fires.lock().unwrap(), 1, "Skip coalesces the backlog");
    let overruns = executor.timer_overruns(id).unwrap();
    assert!(overruns >= 10, "dropped periods counted: {overruns}");

    let mut reported = std::vec::Vec::new();
    executor.drain_violations(|v| reported.push((v.rule, v.measured)));
    let overrun_rows: std::vec::Vec<_> = reported
        .iter()
        .filter(|(r, _)| *r == "timer-overrun-runtime")
        .collect();
    assert_eq!(overrun_rows.len(), 1, "one report: {reported:?}");
    assert_eq!(
        overrun_rows[0].1, overruns,
        "the violation carries the dropped count"
    );

    // A second check with no new stall must stay silent — the rule
    // reports newly dropped activations, not the running total.
    let _ = elapse_then_spin_once(&mut executor, 10);
    let mut again = std::vec::Vec::new();
    executor.drain_violations(|v| again.push(v.rule));
    assert!(
        !again.contains(&"timer-overrun-runtime"),
        "no repeat without a new drop: {again:?}"
    );
}

/// Issue #514 — a detected violation must reach a SINK, not just a ring.
///
/// The ring itself was never the problem: `drain_violations` worked, it
/// simply had no caller outside tests, so every rule ran each spin and
/// discarded its verdict. Logging happens at detection, which is why
/// this test can still drain the ring afterwards — the two paths are
/// independent, and an application that reports violations its own way
/// is unaffected by the default.
#[cfg(feature = "alloc")]
#[test]
fn a_violation_is_logged_and_still_drainable() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    executor
        .register_timer(TimerDuration::from_millis(10), || {})
        .unwrap();

    // Stall past several periods so the overrun rule fires.
    let _ = elapse_then_spin_once(&mut executor, 120);

    // Reporting is on by default...
    let mut drained = std::vec::Vec::new();
    executor.drain_violations(|v| drained.push(v.rule));
    assert!(
        drained.contains(&"timer-overrun-runtime"),
        "the ring still carries the violation: {drained:?}"
    );
    assert_eq!(executor.violations_dropped(), 0);

    // ...and can be turned off by an application that reports its own way.
    executor.set_report_violations(false);
    let _ = elapse_then_spin_once(&mut executor, 120);
    let mut again = std::vec::Vec::new();
    executor.drain_violations(|v| again.push(v.rule));
    assert!(
        again.contains(&"timer-overrun-runtime"),
        "silencing the log must not silence the ring: {again:?}"
    );
}

/// Issue #514 — faults produced faster than they are reported must be
/// COUNTED, not silently lost. The ring holds `MAX_VIOLATIONS`; an
/// application that never drains would otherwise report a stale prefix
/// of its faults and no indication that a prefix is all it is.
#[cfg(feature = "alloc")]
#[test]
fn violations_beyond_the_ring_are_counted() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    // ONE timer, stalled repeatedly — not `MAX_VIOLATIONS + 4` timers.
    //
    // This test asked for 12 callback slots against a `MAX_CBS` that has
    // defaulted to 4 since 2026-03, so `register_timer` returned
    // `ExecutorFull` on the fifth and the `.unwrap()` panicked. It never
    // passed; it was simply never run (see the module note on the `std`
    // feature gate), so the ring-overflow behaviour it exists to prove has
    // been unverified since #514 landed.
    //
    // Overflowing the ring does not need one timer per violation: a single
    // overrunning timer produces a fresh violation on every stalled spin, and
    // nothing here drains, so `MAX_VIOLATIONS + 4` spins overflow a ring of
    // `MAX_VIOLATIONS`. That also keeps the test independent of `MAX_CBS`,
    // which is a build-time knob any consumer may set.
    executor
        .register_timer(TimerDuration::from_millis(10), || {})
        .unwrap();
    for _ in 0..(super::monitor::MAX_VIOLATIONS + 4) {
        let _ = elapse_then_spin_once(&mut executor, 120);
    }
    assert!(
        executor.violations_dropped() >= 1,
        "overflow counted: dropped={}",
        executor.violations_dropped()
    );
}

/// Issue #515 — a period the spin cadence cannot express is announced
/// once, at the point the executor first learns its spin period.
///
/// The audit is a log-side effect, so what is asserted here is the
/// mechanism around it: it runs exactly once, only on a non-zero
/// timeout, and it never touches timer behaviour (the alternation it
/// warns about is legal — the mean cadence is preserved).
#[cfg(feature = "alloc")]
#[test]
fn spin_quantization_audit_runs_once_on_the_first_timed_spin() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    // 33 ms on a 5 ms spin: alternates 35/30.
    executor
        .register_timer(TimerDuration::from_millis(33), || {})
        .unwrap();

    // A zero-timeout spin carries no cadence information, so the audit
    // must not consume its one shot on it.
    let _ = executor.spin_once(core::time::Duration::from_millis(0));
    assert!(
        !executor.spin_quantization_checked,
        "a zero timeout says nothing about the tier's spin period"
    );

    let _ = executor.spin_once(core::time::Duration::from_millis(5));
    assert!(
        executor.spin_quantization_checked,
        "audited on first timed spin"
    );

    // Idempotent: later spins do not re-audit (and so cannot re-warn
    // every cycle for the lifetime of the image).
    let _ = executor.spin_once(core::time::Duration::from_millis(5));
    assert!(executor.spin_quantization_checked);
}

/// Issue #515 — a period that divides the spin period evenly is not a
/// finding, and neither is a timer with no period.
#[cfg(feature = "alloc")]
#[test]
fn spin_quantization_audit_accepts_exact_multiples() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    let id = executor
        .register_timer(TimerDuration::from_millis(50), || {})
        .unwrap();
    let _ = executor.spin_once(core::time::Duration::from_millis(10));
    // 50 ms on a 10 ms spin is expressible; the timer is untouched by
    // the audit either way.
    assert_eq!(executor.timer_period_us(id), Some(50_000));
    assert_eq!(executor.timer_overruns(id), Some(0));
}

/// Issue 0563 — `Executor` must stay small enough that CONSTRUCTING one is not
/// a large stack temporary.
///
/// It is returned by value from `assemble` -> `from_session_in` -> `open_in`
/// before being written into the caller's (static) storage, so its size is paid
/// on the stack of whatever thread opens it. On Zephyr Cortex-M that thread is
/// `main`, and at 11632 bytes it overflowed a 16 KB stack into the idle
/// thread's saved context — issue 0552, which presented as `PC=0` and read for
/// a day as a call through a NULL function pointer.
///
/// Measured breakdown at the time (host, 64-bit):
///
/// ```text
/// Executor          = 11632
///   remap_table     =  6664   <- 57%, the seventh sized table, still inline
///   nodes           =  1448
///   monitor_states  =   128
///   component_slots =   104
///   dispatch_slots  =    72
/// ```
///
/// Moving `remap_table` into the carved backing (where phase-271 had already
/// put the other six) removed that 6664. The bound below is deliberately loose
/// — this guards against a NEW multi-KB inline table appearing, not against
/// small drift, and host `size_of` is not the target's anyway.
#[test]
fn executor_stays_small_enough_to_construct_on_a_stack() {
    let size = core::mem::size_of::<Executor<'static>>();
    assert!(
        size <= 6 * 1024,
        "size_of::<Executor>() = {size} — a new inline table has landed in the \
         struct. It is built as a stack temporary before being moved into the \
         caller's storage, so this is main-stack cost on every embedded board \
         (issue 0563). Carve it from the backing like `entries`/`remap_table` \
         instead of holding it inline."
    );
}

/// Issue 0757 — a take that FAILS must be counted, not swallowed.
///
/// The drain loop had four copies (typed, raw, borrowed/zero-copy, and the C
/// one), and three of them treated any non-`Ok` take as "the queue is empty":
/// `else { break }`. That made a dropped sample indistinguishable from an idle
/// subscription — the executor reported a clean spin, the message was gone, and
/// nothing anywhere counted it.
///
/// The unit suite could not have caught it: `MockSubscriber` held a queue of
/// canned MESSAGES, so no test could express a failing take at all. Injecting
/// the failure is the other half of the fix.
///
/// The assertion is `subscription_errors`, deliberately, rather than the log
/// line: issue 0737 had already fixed the C copy by PROPAGATING to
/// `spin_once`, so the remedy for this class already existed one function over,
/// and a log-only fix would have been a second spelling of it.
#[test]
fn failed_subscription_take_is_counted_not_swallowed() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("failed_take_is_counted")
        .build()
        .unwrap();
    let count = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count_cb = count.clone();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/lossy", move |_msg: &TestMsg| {
            count_cb.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        })
        .unwrap();

    let arena_ptr = executor.arena.as_ptr() as *const u8;
    let off = executor.entries[0].as_ref().unwrap().offset;
    unsafe { &*(arena_ptr.add(off) as *const MockSubscriber) }
        .load_error(nros_rmw::TransportError::MessageTooLarge);

    let result = executor.spin_once(core::time::Duration::from_millis(0));

    assert_eq!(
        result.subscription_errors, 1,
        "a failing take must reach subscription_errors — the pre-0757 drain \
         loop broke out of the loop on any non-Ok take, so this read as an \
         empty queue and the spin reported clean"
    );
    assert!(
        result.any_errors(),
        "a spin that dropped a sample is not a clean spin"
    );
    assert_eq!(
        count.load(std::sync::atomic::Ordering::SeqCst),
        0,
        "no callback can run for a message that never arrived"
    );
}

/// Issue 0757, the other direction — an EMPTY subscription is still not an
/// error.
///
/// Negative control for the test above. The failure mode being fixed is
/// conflating "take failed" with "nothing to take", and a fix that counted both
/// would be the same conflation with the sign flipped: every idle spin on every
/// board would report errors, which is how a counter stops being read.
#[test]
fn empty_subscription_is_not_an_error() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);

    let nid = executor
        .node_builder("empty_is_not_an_error")
        .build()
        .unwrap();
    executor
        .node_mut(nid)
        .create_subscription::<TestMsg, _>("/idle", move |_msg: &TestMsg| {})
        .unwrap();

    let result = executor.spin_once(core::time::Duration::from_millis(0));

    assert_eq!(
        result.subscription_errors, 0,
        "an idle subscription must not count as an error"
    );
    assert!(!result.any_errors(), "an idle spin is a clean spin");
}

/// Phase 376 W5/B1 — `create_node` REGISTERS, and deduplicates by name.
///
/// Until 2026-08-24 it registered nothing: it built a `NodeHandle` and returned
/// it, so the executor had never heard of a node the caller had just created.
/// The `create_node` VTABLE slot's contract is that the runtime calls it once
/// per distinct `(name, namespace)`, and with no registry to consult there is
/// nothing to make that true — every backend would need its own dedup, which is
/// the registry the slot exists to delete.
#[test]
fn create_node_registers_and_dedups_by_name() {
    let mut executor: Executor = executor_with_clock(MockSession::new());

    assert!(
        executor
            .node_id_by_name("talker", executor.namespace.as_str())
            .is_none(),
        "precondition: the table starts without this node"
    );

    let ns: heapless::String<64> = executor.namespace.clone();
    drop(executor.create_node("talker").unwrap());
    let first = executor
        .node_id_by_name("talker", ns.as_str())
        .expect("create_node must put the node in the table");

    // The second call is the one that used to hand out an unregistered
    // duplicate. It must reuse the record, not push a second one.
    drop(executor.create_node("talker").unwrap());
    let second = executor
        .node_id_by_name("talker", ns.as_str())
        .expect("still registered");
    assert_eq!(
        first, second,
        "a repeated name must reuse its record, not create a second"
    );

    // A DIFFERENT name is a different node and does get its own record.
    drop(executor.create_node("listener").unwrap());
    let other = executor
        .node_id_by_name("listener", ns.as_str())
        .expect("a distinct name registers separately");
    assert_ne!(first, other, "distinct names are distinct nodes");
}

// ============================================================================
// Shutdown hooks (issue 0790)
// ============================================================================

// The hook context is a raw pointer the executor holds for its whole life, so
// every fixture below is `&'static` via a leaked box. `alloc`, not `std`: this
// crate is `no_std` and the test module inherits that.
use alloc::{boxed::Box, vec::Vec};

/// The ordering test's shared observation point.
///
/// A hook is an `extern "C" fn(*mut c_void)` — it gets its own context and
/// nothing else — so what it can SAY about the world has to be reachable
/// through that pointer. Each hook is handed one of these and stamps it with
/// the two numbers that make the ORDER checkable rather than just the fact that
/// it ran: how many times the session had been closed when it fired, and its
/// position in a global run sequence.
#[derive(Debug, Default)]
struct HookWitness {
    /// How many times this hook ran. The whole point of "exactly once".
    runs: core::sync::atomic::AtomicUsize,
    /// `closes` observed at the moment it ran. `0` for a pre-shutdown hook
    /// means the session was still open; `1` for an on-shutdown hook means the
    /// close already happened.
    closes_seen: core::sync::atomic::AtomicUsize,
    /// Its ticket from the shared run sequence, so "pre before post" is an
    /// assertion and not an inference.
    sequence: core::sync::atomic::AtomicUsize,
}

/// What every hook in these tests reads and writes. `&'static` because the
/// registration hands the executor a raw pointer that must outlive it, and a
/// leaked box is the honest way to say so in a test.
struct ShutdownFixture {
    closes: &'static core::sync::atomic::AtomicUsize,
    ticket: core::sync::atomic::AtomicUsize,
    pre: HookWitness,
    post: HookWitness,
}

impl ShutdownFixture {
    /// A fixture plus the `MockSession` whose `close()` feeds it. Leaked, not
    /// dropped: both the session and the executor's hook table hold references
    /// into it for as long as the executor lives, and the test's assertions run
    /// after the executor is gone.
    fn new() -> (&'static ShutdownFixture, MockSession) {
        let closes: &'static core::sync::atomic::AtomicUsize =
            Box::leak(Box::new(core::sync::atomic::AtomicUsize::new(0)));
        let fixture: &'static ShutdownFixture = Box::leak(Box::new(ShutdownFixture {
            closes,
            ticket: core::sync::atomic::AtomicUsize::new(0),
            pre: HookWitness::default(),
            post: HookWitness::default(),
        }));
        (fixture, MockSession::with_close_observer(closes))
    }

    fn closes(&self) -> usize {
        self.closes.load(core::sync::atomic::Ordering::SeqCst)
    }

    fn record(&self, witness: &HookWitness) {
        use core::sync::atomic::Ordering::SeqCst;
        witness.runs.fetch_add(1, SeqCst);
        witness.closes_seen.store(self.closes(), SeqCst);
        witness
            .sequence
            .store(self.ticket.fetch_add(1, SeqCst) + 1, SeqCst);
    }

    fn as_ctx(&'static self) -> *mut core::ffi::c_void {
        self as *const ShutdownFixture as *mut core::ffi::c_void
    }
}

fn runs(witness: &HookWitness) -> usize {
    witness.runs.load(core::sync::atomic::Ordering::SeqCst)
}

fn closes_seen(witness: &HookWitness) -> usize {
    witness
        .closes_seen
        .load(core::sync::atomic::Ordering::SeqCst)
}

fn sequence(witness: &HookWitness) -> usize {
    witness.sequence.load(core::sync::atomic::Ordering::SeqCst)
}

/// # Safety
/// `ctx` must be the `&'static ShutdownFixture` the registration passed.
unsafe extern "C" fn record_pre(ctx: *mut core::ffi::c_void) {
    let fixture = unsafe { &*(ctx as *const ShutdownFixture) };
    fixture.record(&fixture.pre);
}

/// # Safety
/// `ctx` must be the `&'static ShutdownFixture` the registration passed.
unsafe extern "C" fn record_post(ctx: *mut core::ffi::c_void) {
    let fixture = unsafe { &*(ctx as *const ShutdownFixture) };
    fixture.record(&fixture.post);
}

/// # Safety
/// `ctx` must be a `&'static AtomicUsize`.
unsafe extern "C" fn bump(ctx: *mut core::ffi::c_void) {
    let counter = unsafe { &*(ctx as *const core::sync::atomic::AtomicUsize) };
    counter.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
}

/// Issue 0790 — the ORDER is the feature, so this is the test that has to hold.
///
/// It asserts four separate things, because three of them pass on an
/// implementation that has the ordering backwards:
///
/// * the pre-shutdown hook ran while the session was still OPEN (`closes_seen
///   == 0`) — this is the half with no workaround, the one a node needs to
///   publish a final state or park an actuator;
/// * the on-shutdown hook ran AFTER the close (`closes_seen == 1`);
/// * the pre hook's sequence ticket precedes the post hook's;
/// * each ran exactly once, including across a SECOND `close()` and the drop
///   that follows.
#[test]
fn a_pre_shutdown_hook_sees_a_live_session_and_an_on_shutdown_hook_sees_a_closed_one() {
    let (fixture, session) = ShutdownFixture::new();
    let mut executor: Executor = executor_with_clock(session);

    // SAFETY: `fixture` is `&'static`, so the context outlives the executor,
    // and both trampolines expect exactly that pointer.
    let pre_handle = unsafe {
        executor
            .add_pre_shutdown_callback(record_pre, fixture.as_ctx())
            .expect("a fresh executor has a free pre-shutdown slot")
    };
    let post_handle = unsafe {
        executor
            .add_on_shutdown_callback(record_post, fixture.as_ctx())
            .expect("a fresh executor has a free on-shutdown slot")
    };
    assert_eq!(pre_handle.phase(), Some(ShutdownPhase::Pre));
    assert_eq!(post_handle.phase(), Some(ShutdownPhase::Post));
    assert_eq!(executor.shutdown_callback_count(ShutdownPhase::Pre), 1);
    assert_eq!(executor.shutdown_callback_count(ShutdownPhase::Post), 1);

    // Nothing has run yet: registration is not invocation.
    assert_eq!(runs(&fixture.pre), 0, "pre-shutdown hook ran too early");
    assert_eq!(runs(&fixture.post), 0, "on-shutdown hook ran too early");

    executor.close().expect("MockSession::close succeeds");

    assert_eq!(runs(&fixture.pre), 1, "pre-shutdown hook did not run");
    assert_eq!(runs(&fixture.post), 1, "on-shutdown hook did not run");
    assert_eq!(
        closes_seen(&fixture.pre),
        0,
        "the pre-shutdown hook ran AFTER the session closed — that is the phase \
         with no workaround, and it is useless if the entities are already gone"
    );
    assert_eq!(
        closes_seen(&fixture.post),
        1,
        "the on-shutdown hook ran BEFORE the session closed"
    );
    assert!(
        sequence(&fixture.pre) < sequence(&fixture.post),
        "pre-shutdown must precede on-shutdown (pre ticket {}, post ticket {})",
        sequence(&fixture.pre),
        sequence(&fixture.post),
    );

    // Exactly once, part 1: a second close finds both tables empty.
    executor.close().expect("a second close is not an error");
    assert_eq!(runs(&fixture.pre), 1, "pre-shutdown hook ran twice");
    assert_eq!(runs(&fixture.post), 1, "on-shutdown hook ran twice");

    // Exactly once, part 2: the drop sweep must not re-run what close ran.
    drop(executor);
    assert_eq!(runs(&fixture.pre), 1, "drop re-ran the pre-shutdown hook");
    assert_eq!(runs(&fixture.post), 1, "drop re-ran the on-shutdown hook");
}

/// Issue 0790 — the C API's `nros_executor_fini` drops the executor in place
/// and never calls `close()`. If `Drop` did not sweep the tables the whole
/// facility would be silently inert for every C entry, which is the failure
/// mode this repo calls a museum binary: the code is there, the gate is green,
/// and nothing runs.
#[test]
fn dropping_an_executor_without_closing_it_still_runs_both_phases_once() {
    let (fixture, session) = ShutdownFixture::new();
    let mut executor: Executor = executor_with_clock(session);
    // SAFETY: as above — `fixture` is `&'static`.
    unsafe {
        executor
            .add_pre_shutdown_callback(record_pre, fixture.as_ctx())
            .unwrap();
        executor
            .add_on_shutdown_callback(record_post, fixture.as_ctx())
            .unwrap();
    }

    drop(executor);

    assert_eq!(runs(&fixture.pre), 1, "drop skipped the pre-shutdown hook");
    assert_eq!(runs(&fixture.post), 1, "drop skipped the on-shutdown hook");
    assert!(
        sequence(&fixture.pre) < sequence(&fixture.post),
        "drop must keep the phase order"
    );
}

/// Issue 0790 — the handle exists so a registration can be undone.
#[test]
fn a_removed_shutdown_hook_does_not_run() {
    let (fixture, session) = ShutdownFixture::new();
    let mut executor: Executor = executor_with_clock(session);
    // SAFETY: as above.
    let handle = unsafe {
        executor
            .add_pre_shutdown_callback(record_pre, fixture.as_ctx())
            .unwrap()
    };

    assert!(
        executor.remove_pre_shutdown_callback(handle),
        "removing a live hook reports true"
    );
    assert_eq!(executor.shutdown_callback_count(ShutdownPhase::Pre), 0);
    assert!(
        !executor.remove_pre_shutdown_callback(handle),
        "removing it twice reports false rather than erroring"
    );

    executor.close().unwrap();
    assert_eq!(runs(&fixture.pre), 0, "a removed hook still ran");
}

/// Issue 0790 — a slot index means nothing without the table it indexes, and
/// there are two tables. The phase tag in the handle is what keeps a pre-phase
/// handle from removing the on-phase hook that happens to share its index.
#[test]
fn a_handle_from_one_phase_cannot_remove_the_other_phases_hook() {
    let (fixture, session) = ShutdownFixture::new();
    let mut executor: Executor = executor_with_clock(session);
    // SAFETY: as above.
    let (pre_handle, post_handle) = unsafe {
        (
            executor
                .add_pre_shutdown_callback(record_pre, fixture.as_ctx())
                .unwrap(),
            executor
                .add_on_shutdown_callback(record_post, fixture.as_ctx())
                .unwrap(),
        )
    };
    // Same slot index, different phase — the exact collision an untagged
    // index-only handle would not survive.
    assert_eq!(pre_handle.index(), post_handle.index());
    assert_ne!(pre_handle, post_handle);

    assert!(!executor.remove_on_shutdown_callback(pre_handle));
    assert!(!executor.remove_pre_shutdown_callback(post_handle));
    assert_eq!(executor.shutdown_callback_count(ShutdownPhase::Pre), 1);
    assert_eq!(executor.shutdown_callback_count(ShutdownPhase::Post), 1);

    executor.close().unwrap();
    assert_eq!(runs(&fixture.pre), 1);
    assert_eq!(runs(&fixture.post), 1);
}

/// Issue 0790 — the tables are fixed-capacity by design (issue 0460's
/// precedent: a static slot is a cost every image pays). Overflow must name the
/// knob rather than silently dropping the registration, because a dropped
/// shutdown hook is invisible until the day it was needed.
#[test]
fn registering_past_capacity_reports_the_knob_and_frees_on_removal() {
    let session = MockSession::new();
    let mut executor: Executor = executor_with_clock(session);
    let counter: &'static core::sync::atomic::AtomicUsize =
        Box::leak(Box::new(core::sync::atomic::AtomicUsize::new(0)));
    let ctx = counter as *const _ as *mut core::ffi::c_void;

    let mut handles = Vec::new();
    for slot in 0..crate::config::MAX_SHUTDOWN_CBS {
        // SAFETY: `counter` is `&'static` and `bump` expects exactly it.
        handles.push(unsafe {
            executor
                .add_pre_shutdown_callback(bump, ctx)
                .unwrap_or_else(|e| panic!("slot {slot} should be free, got {e:?}"))
        });
    }
    // SAFETY: as above.
    let overflow = unsafe { executor.add_pre_shutdown_callback(bump, ctx) };
    assert_eq!(
        overflow.unwrap_err(),
        NodeError::ShutdownCallbacksFull,
        "overflow must name the shutdown-hook table, not a generic full error"
    );

    // A freed slot is reusable — the tables are arrays with holes, not a fill
    // cursor that can only grow.
    assert!(executor.remove_pre_shutdown_callback(handles[0]));
    // SAFETY: as above.
    let reused = unsafe { executor.add_pre_shutdown_callback(bump, ctx) }
        .expect("a removed slot is available again");
    assert_eq!(reused, handles[0], "the freed slot is the one reused");

    executor.close().unwrap();
    assert_eq!(
        counter.load(core::sync::atomic::Ordering::SeqCst),
        crate::config::MAX_SHUTDOWN_CBS,
        "every occupied slot runs exactly once"
    );
}

// ============================================================================
// Issue 0796 — the raw action server's post-accept hook
// ============================================================================

/// How many times the accepted-goal hook fired, and for which goal.
///
/// Statics rather than a captured counter because the hook is an
/// `extern "C" fn` — the same constraint that makes the C and C++ tiers pass a
/// context pointer. The test is single-threaded and resets both up front.
static ACCEPTED_CALLS: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
static ACCEPTED_UUID_FIRST_BYTE: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(0);

unsafe extern "C" fn defer_every_goal(
    _goal_id: *const nros_core::GoalId,
    _data: *const u8,
    _len: usize,
    _ctx: *mut core::ffi::c_void,
) -> nros_core::GoalResponse {
    nros_core::GoalResponse::AcceptAndDefer
}

unsafe extern "C" fn reject_every_goal(
    _goal_id: *const nros_core::GoalId,
    _data: *const u8,
    _len: usize,
    _ctx: *mut core::ffi::c_void,
) -> nros_core::GoalResponse {
    nros_core::GoalResponse::Reject
}

unsafe extern "C" fn refuse_every_cancel(
    _goal_id: *const nros_core::GoalId,
    _status: nros_core::GoalStatus,
    _ctx: *mut core::ffi::c_void,
) -> nros_core::CancelResponse {
    nros_core::CancelResponse::Reject
}

unsafe extern "C" fn count_accepted(
    goal_id: *const nros_core::GoalId,
    _ctx: *mut core::ffi::c_void,
) {
    use core::sync::atomic::Ordering::SeqCst;
    ACCEPTED_CALLS.fetch_add(1, SeqCst);
    ACCEPTED_UUID_FIRST_BYTE.store(unsafe { (*goal_id).uuid[0] }, SeqCst);
}

/// A `send_goal` wire frame: `[CDR-LE header][fixed uint8[16] goal_id]`.
/// The goal FIELDS would follow; this action's callbacks ignore them.
#[cfg(test)]
fn mk_send_goal_req(goal_id: &nros_core::GoalId) -> ([u8; 256], usize) {
    let mut b = [0u8; 256];
    b[..4].copy_from_slice(&[0x00, 0x01, 0x00, 0x00]);
    b[4..20].copy_from_slice(&goal_id.uuid);
    (b, 20)
}

/// Issue 0796 — a goal accepted with `AcceptAndDefer` must reach the
/// accepted-goal hook exactly once.
///
/// This is the mechanism the C++ callback tier was missing: it registered
/// `accepted_callback: None`, so a C++ user who deferred a goal was never told
/// it had been accepted — the moment they are supposed to start executing it.
/// The hook is registered unconditionally now and dispatched here, in
/// `action_server_raw_try_process`.
///
/// Three things are pinned: it fires (once, for the right goal), it fires
/// AFTER the accept reply is on the wire (a long-running body must not delay
/// acceptance), and a REJECTED goal does not reach it at all.
#[test]
fn a_deferred_goal_reaches_the_accepted_callback_exactly_once() {
    use super::{
        action_core::ActionServerCore,
        arena::{ActionServerRawArenaEntry, action_server_raw_try_process},
    };
    use crate::mock::MockPublisher;
    use core::sync::atomic::Ordering::SeqCst;
    use nros_core::GoalId;

    const GB: usize = 128;
    const RB: usize = 128;
    const FB: usize = 128;
    const MG: usize = 2;

    ACCEPTED_CALLS.store(0, SeqCst);
    ACCEPTED_UUID_FIRST_BYTE.store(0, SeqCst);

    let core: ActionServerCore<GB, RB, FB, MG> = ActionServerCore::from_channels(
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockServiceServer::new(),
        MockPublisher,
        MockPublisher,
    );
    let mut entry: ActionServerRawArenaEntry<GB, RB, FB, MG> = ActionServerRawArenaEntry {
        core,
        goal_callback: defer_every_goal,
        cancel_callback: refuse_every_cancel,
        accepted_callback: Some(count_accepted),
        context: core::ptr::null_mut(),
    };
    let ptr = (&mut entry) as *mut _ as *mut u8;

    // No request pending → no work, and certainly no accept.
    let did_work = unsafe { action_server_raw_try_process::<GB, RB, FB, MG>(ptr, 0, 0) }.unwrap();
    assert!(!did_work, "an idle server must report no work");
    assert_eq!(ACCEPTED_CALLS.load(SeqCst), 0);

    let g = GoalId { uuid: [0xA5u8; 16] };
    let (req, len) = mk_send_goal_req(&g);
    entry.core.send_goal_server.load(req, len);

    let did_work = unsafe { action_server_raw_try_process::<GB, RB, FB, MG>(ptr, 0, 0) }.unwrap();
    assert!(did_work);
    assert_eq!(
        ACCEPTED_CALLS.load(SeqCst),
        1,
        "a deferred goal must reach the accepted callback exactly once"
    );
    assert_eq!(
        ACCEPTED_UUID_FIRST_BYTE.load(SeqCst),
        0xA5,
        "the hook must be told WHICH goal was accepted"
    );

    // The accept reply is on the wire, and the goal is live in the arena —
    // i.e. the hook ran after acceptance, not instead of it.
    {
        let sent = entry.core.send_goal_server.sent.borrow();
        assert_eq!(sent.len(), 1, "exactly one accept reply");
        assert_eq!(sent[0].1[4], 1, "the reply must say ACCEPTED");
    }
    assert_eq!(entry.core.active_goals.len(), 1);

    // Spinning again with nothing pending must not re-fire the hook.
    let did_work = unsafe { action_server_raw_try_process::<GB, RB, FB, MG>(ptr, 0, 0) }.unwrap();
    assert!(!did_work);
    assert_eq!(
        ACCEPTED_CALLS.load(SeqCst),
        1,
        "the hook is per-acceptance, not per-spin"
    );

    // A REJECTED goal must not reach it.
    entry.goal_callback = reject_every_goal;
    let g2 = GoalId { uuid: [0x5Au8; 16] };
    let (req2, len2) = mk_send_goal_req(&g2);
    entry.core.send_goal_server.load(req2, len2);
    let did_work = unsafe { action_server_raw_try_process::<GB, RB, FB, MG>(ptr, 0, 0) }.unwrap();
    assert!(did_work);
    assert_eq!(
        ACCEPTED_CALLS.load(SeqCst),
        1,
        "a rejected goal must not reach the accepted callback"
    );
    assert_eq!(
        entry.core.active_goals.len(),
        1,
        "still only the first goal"
    );
}

// ====================================================================
// Phase 403 step 3 -- the ARENA COST OF ONE ENTITY, MEASURED
//
// Step 3 derives the arena as a sum over what the entity inventory says
// the image creates, so it needs a per-kind cost. These MEASURE it:
// `arena_alloc` is a BUMP allocator, so an `arena_used()` delta across
// exactly one registration IS that entity's arena footprint, including
// alignment padding at the region boundary and the trailing buffer that
// `arena_alloc_with_trailing` places after the entry.
//
// Measured, never modelled. A table derived by reading `size_of` on the
// entry structs would agree with the allocator right up until an
// alignment or a trailing term moved, and this campaign has produced six
// mechanisms that were arithmetically right and did not reach the thing
// they sized. The derivation must be checked against what the allocator
// ACTUALLY claims, which is what these deltas are.
//
// WHY THIS MATTERS MORE THAN THE OTHER KNOBS. An under-sized arena halts
// DURING entity creation, before the first spin, so issue 0900's advisory
// never prints. `MAX_CBS` fails at registration with `ExecutorFull` and
// `MAX_NODES` with `NodeTableFull`, both naming their knob; the arena is
// the one number whose failure mode cannot report itself. So its inputs
// are measured before anything derives from them.
// ====================================================================

/// The arena cost of one TIMER, which carries no payload buffer and so is
/// the floor every other kind is measured against.
fn arena_cost_timer(executor: &mut Executor<'_>) -> usize {
    let before = executor.arena_used();
    executor
        .register_timer(TimerDuration::from_millis(1000), || {})
        .expect("timer registration must succeed");
    executor.arena_used() - before
}

/// The arena cost of one SERVICE SERVER. A service carries TWO payload
/// buffers (request and reply), so it is the kind whose cost is least
/// predictable from a subscription's.
fn arena_cost_service(executor: &mut Executor<'_>, name: &str) -> usize {
    let before = executor.arena_used();
    executor
        .register_service_sized::<TestService, _, 512, 512>(name, |req: &TestServiceRequest| {
            TestServiceReply { sum: req.a }
        })
        .expect("service registration must succeed");
    executor.arena_used() - before
}

#[test]
fn a_service_server_costs_a_measurable_and_constant_arena_slice() {
    let mut executor: Executor = executor_with_clock(MockSession::new());
    let first = arena_cost_service(&mut executor, "/svc_a");
    assert!(
        first > 0,
        "a service server must claim arena; a zero delta means the probe is \
         not measuring the allocation it names"
    );
    let second = arena_cost_service(&mut executor, "/svc_b");
    assert_eq!(
        first, second,
        "two service servers must cost the same; step 3 sums a per-kind \
         constant over the declared count, which is only valid if it IS \
         constant"
    );
}

#[test]
fn an_unbounded_subscription_costs_more_than_a_bounded_one() {
    // Measured 2316 bounded against 5356 unbounded on x86_64 -- 2.3x. This is
    // why step 3 must REFUSE when a subscribed type is unbounded rather than
    // budgeting the fallback: the fallback is the expensive arm, so an image
    // that silently took it would be sized for a number nobody chose.
    let mut executor: Executor = executor_with_clock(MockSession::new());
    let nid = executor
        .node_builder("bounded_vs_not")
        .build()
        .expect("node");
    let bounded = arena_delta(&mut executor, nid, "/b", qos_with_depth(1), true);
    let unbounded = arena_delta(&mut executor, nid, "/u", qos_with_depth(1), false);
    assert!(
        unbounded > bounded,
        "the unbounded fallback must cost MORE than a derived bound \
         (got {unbounded} vs {bounded}); if it were cheaper, refusing to \
         derive would be the conservative choice and it is not"
    );
}

/// The per-kind arena cost table, as a MEASUREMENT PROBE.
///
///     cargo test -p nros-node --features std --lib report_arena_costs -- --ignored
///
/// Reports through `panic!` rather than `println!` deliberately: this crate is
/// `#![no_std]`, `std::println!` is banned in its `src/` (issue 0589 -- it
/// SIGSEGVs a Zephyr native_sim image), and the panic payload is the one
/// output channel that needs no stdio. `#[ignore]` because it is a tool for a
/// human, not a verdict; the assertions below are the verdicts and they run
/// every time.
///
/// HOST NUMBERS. The entry structs hold pointers, so a 32-bit target's table
/// differs. Step 3 must compute per target and may never copy these.
///
/// Measured 2026-09-03, x86_64:
///     timer=32  sub_d1=2316  sub_d10=2504  sub_unbounded=5356
///     service_512_512=3496  per_depth=20
///
/// The d1->d10 delta of 188 does NOT match
/// `(depth + 1) * bound + (depth + 1) * 8` exactly for any plausible bound of
/// `TestMsg` (a bound of 12 predicts 180), so the region carries alignment or
/// padding terms that formula omits. That gap is the whole reason step 3
/// derives against THIS probe rather than against the formula.
#[test]
#[ignore = "measurement probe: run with --ignored to read the table"]
fn report_arena_costs() {
    // One executor PER measurement. The default MAX_CBS is 4, and this probe
    // registers more entities than that -- the first version died with
    // `ExecutorFull`, which is the knob naming itself and is exactly the
    // failure mode the arena does NOT have. A bump allocator's deltas are
    // independent, so per-kind executors measure the same thing.
    let timer = {
        let mut e: Executor = executor_with_clock(MockSession::new());
        arena_cost_timer(&mut e)
    };
    let (d1, d10, unb) = {
        let mut e: Executor = executor_with_clock(MockSession::new());
        let nid = e.node_builder("arena_report").build().expect("node");
        let a = arena_delta(&mut e, nid, "/r1", qos_with_depth(1), true);
        let b = arena_delta(&mut e, nid, "/r10", qos_with_depth(10), true);
        let c = arena_delta(&mut e, nid, "/ru", qos_with_depth(1), false);
        (a, b, c)
    };
    let svc = {
        let mut e: Executor = executor_with_clock(MockSession::new());
        arena_cost_service(&mut e, "/rsvc")
    };
    panic!(
        "ARENA timer={timer} sub_d1={d1} sub_d10={d10} sub_unbounded={unb} \
         service_512_512={svc} per_depth={}",
        (d10 as isize - d1 as isize) / 9
    );
}

#[test]
fn a_timer_costs_a_measurable_and_depth_independent_arena_slice() {
    let mut executor: Executor = executor_with_clock(MockSession::new());
    let first = arena_cost_timer(&mut executor);
    assert!(
        first > 0,
        "a timer must claim arena; a zero delta means the probe is not \
         measuring the allocation it names"
    );

    // A second timer costs the same: the per-kind cost is a CONSTANT the
    // derivation may multiply by a declared count. If this ever differs,
    // the arena is not a straight sum and step 3's premise is wrong.
    let second = arena_cost_timer(&mut executor);
    assert_eq!(
        first, second,
        "two timers must cost the same; step 3 sums a per-kind constant \
         over the declared count, which is only valid if it IS constant"
    );
}

#[test]
fn a_subscriptions_arena_cost_scales_with_declared_depth() {
    // The whole reason step 2 had to land first. Depth is a MULTIPLIER on
    // the bound -- `buffered_region_size(depth, bound)` is
    // `(depth + 1) * bound + (depth + 1) * 8` -- so a derivation that
    // ignores it is wrong by up to 10x, in whichever direction hurts.
    let mut executor: Executor = executor_with_clock(MockSession::new());
    let nid = executor.node_builder("arena_probe").build().expect("node");

    let d1 = arena_delta(&mut executor, nid, "/d1", qos_with_depth(1), true);
    let d10 = arena_delta(&mut executor, nid, "/d10", qos_with_depth(10), true);

    assert!(
        d10 > d1,
        "depth 10 must claim more arena than depth 1 (got {d10} vs {d1}); \
         if these are equal the buffered region is not depth-scaled and \
         the arena cannot be derived from a declared depth"
    );
}

/// Issue #515's audit is STATIC: it warns when a declared period is not a
/// multiple of the spin period, and says in its own comment that "the jitter
/// stays invisible until someone measures cadence on target". These cover the
/// measurement that makes it visible.
#[test]
fn release_jitter_starts_empty_and_clears() {
    let mut executor: Executor = executor_with_clock(MockSession::new());
    assert_eq!(
        executor.release_jitter(),
        (0, 0, 0),
        "nothing has been measured yet, so it must report nothing rather than \
         a confident zero-jitter"
    );
    executor.clear_release_jitter_stats();
    assert_eq!(executor.release_jitter(), (0, 0, 0));
}

/// The measurement must live on the path the C++ entry actually drives.
/// nros-cpp paces its tiers with a `spin_once` loop, not `spin_period`, so a
/// probe on `spin_period` alone records zero forever on that lane -- issue
/// 0736's mistake one layer up.
#[test]
fn spin_once_counts_wakes_without_spin_period() {
    let mut executor: Executor = executor_with_clock(MockSession::new());
    for _ in 0..5 {
        executor.spin_once(core::time::Duration::from_millis(2));
    }
    let (_max, late, total) = executor.release_jitter();
    assert!(
        total >= 4,
        "five spin_once calls must count at least four intervals, got {total}"
    );
    assert!(late <= total);
}

/// A zero timeout claims no cadence, so it must not enter the statistic --
/// otherwise every `Future::wait` busy spin would register as a late wake.
#[test]
fn a_zero_timeout_spin_claims_no_cadence() {
    let mut executor: Executor = executor_with_clock(MockSession::new());
    for _ in 0..5 {
        executor.spin_once(core::time::Duration::ZERO);
    }
    assert_eq!(
        executor.release_jitter(),
        (0, 0, 0),
        "a polling spin declares no period, so it cannot be late for one"
    );
}

/// `spin_period` must actually COUNT its wakes. Before this, `next_us` and
/// `now_us()` were both in hand every cycle and the difference was discarded,
/// so a loop that never met its period looked identical to one that always did.
#[test]
fn spin_period_counts_its_wakes_and_keeps_late_within_total() {
    let mut executor: Executor = executor_with_clock(MockSession::new());
    let flag = executor.halt_flag();
    std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(60));
        flag.store(true, portable_atomic::Ordering::SeqCst);
    });
    executor
        .spin_period(core::time::Duration::from_millis(2))
        .expect("a clock is installed, so spin_period must run");

    let (max_us, late, total) = executor.release_jitter();
    assert!(total > 0, "the loop ran, so wakes must have been counted");
    assert!(
        late <= total,
        "late wakes ({late}) cannot exceed total wakes ({total})"
    );
    // Deliberately NOT asserting max_us > 0: a host that always meets a 2 ms
    // period is a legitimate outcome, and a test that demanded lateness would
    // fail on the machine behaving best.
    assert!(
        max_us == 0 || late > 0,
        "a non-zero maximum ({max_us} us) with zero late wakes is contradictory"
    );
}
