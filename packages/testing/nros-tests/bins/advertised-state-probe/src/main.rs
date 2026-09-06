//! phase-433 W6 acceptance fixture — print what we ADVERTISE about ourselves,
//! so a stock ROS 2 peer can be asked the same questions about the same
//! entities.
//!
//! Four vtable slot families are `produced` per `just check
//! rmw-slot-producers` and have never been exercised against a live peer:
//! matched counts, the publisher GID, the actual-QoS read-back (issue 0823)
//! and `get_serialization_format`. Phase-381 is why "produced" is not
//! evidence: twelve graph slots were produced, mutation-tested and
//! parity-clean, and the feature did not work at all (issue 0903). Every one
//! of those checks compared our code against our own builders.
//!
//! ## Why this drives the C ABI directly, and what that costs
//!
//! There is no higher layer to drive. `publisher_count_matched_subscriptions`,
//! `subscription_count_matched_publishers` and `get_gid_for_publisher` have NO
//! consumer anywhere in the tree outside `tests/graph_counts.cpp` — no Rust
//! `Publisher` method, no `nros_publisher_*` C entry, no dispatcher in
//! `nros-rmw-cffi`. `publisher_get_actual_qos` has exactly one caller
//! (`CffiSession::create_publisher`), which logs a downgrade warning and
//! throws the profile away; the subscription and the four service/client
//! halves have none. So the layer where these slots are observable IS the
//! vtable, and a probe that went through `Executor` could not reach them at
//! all — the `rmw_publisher_t` it would need is private to the shim.
//!
//! The cost is that the entities here are not created by
//! `Executor::create_node` / `create_publisher`. That matters less than it
//! sounds: those calls reach exactly these slots with exactly these
//! arguments, so the DDS entities are indistinguishable to a peer, which is
//! the premise of `tests/ros2_pub.cpp` and its passing `ros2 topic echo`
//! e2e. What it does mean is that this probe proves the SLOTS against a live
//! peer, not the API path above them — and today there is no API path above
//! them to prove.
//!
//! ## Cyclone only, deliberately
//!
//! Cyclone is the one backend that fills these slots.
//! `RustBackendAdapter::VTABLE` (which is how zenoh reaches the vtable) ends
//! `..EMPTY_VTABLE` and names none of them; `nros-rmw-xrce`'s designated
//! initialiser stops before them; uORB's positional one does too. A zenoh
//! cell here would assert against four NULL pointers.
//!
//! ## The QoS the probe asks for, and why it asks for that one
//!
//! A ZERO-FILLED profile, which is `NROS_RMW_QOS_PROFILE_SYSTEM_DEFAULT`:
//! reliability, durability and history all `SYSTEM_DEFAULT` (0) and depth 0.
//! Cyclone's `make_dds_qos` resolves every one of those before the entity
//! exists — `SYSTEM_DEFAULT` reliability becomes RELIABLE (issue 0829),
//! durability becomes VOLATILE, history becomes KEEP_LAST, and a KEEP_LAST
//! depth of 0 is CLAMPED to 1 because Cyclone's `validate_history_qospolicy`
//! rejects depth < 1 outright. So requested and granted differ in four fields
//! at once, by construction and without needing a peer to disagree with us.
//!
//! That is what makes the read-back falsifiable. If `publisher_get_actual_qos`
//! echoed the request — which is the state issue 0823 found the runtime in —
//! every field printed below would come back 0/SYSTEM_DEFAULT, while the peer
//! reports RELIABLE / VOLATILE / KEEP_LAST (1). The test compares all three.
//!
//! ## Environment
//!
//! | var | default | meaning |
//! | --- | --- | --- |
//! | `ROS_DOMAIN_ID` | `0` | the Cyclone domain. Read explicitly — nothing here reads it for us (issue 0927 is the same trap one fixture over). |
//! | `ADV_PROBE_PUB_TOPIC` | `nros_advertised_pub` | topic the probe PUBLISHES on; a peer subscribing here moves `publisher_count_matched_subscriptions`. |
//! | `ADV_PROBE_SUB_TOPIC` | `nros_advertised_sub` | topic the probe SUBSCRIBES to; a peer publishing here moves `subscription_count_matched_publishers`. |
//! | `ADV_PROBE_NODE` | `advertised_state_probe` | node name the peer sees. |
//! | `ADV_PROBE_HOLD_MS` | `90000` | how long to hold the entities open after READY. |
//! | `ADV_PROBE_TICK_MS` | `200` | publish + count-sample period. |
//!
//! Two topics rather than one, deliberately: a writer and a reader on the SAME
//! topic in ONE participant match EACH OTHER, so `publisher_count_matched_
//! subscriptions` would start at 1 and the "rises from zero" edge would be
//! unobservable. `tests/graph_counts.cpp` relies on exactly that self-match;
//! this probe needs the opposite.

use std::{
    ffi::CString,
    time::{Duration, Instant},
};

use nros_rmw_cffi::{
    NROS_RMW_RET_OK, NrosRmwVtable, nros_rmw_cffi_lookup, rmw_byte_span_t, rmw_gid_t,
    rmw_message_type_support_t, rmw_node_t, rmw_publisher_t, rmw_qos_profile_t, rmw_session_t,
    rmw_subscription_t,
};
use nros_serdes::schema::{Field, FieldType};

/// The DDS-mangled type name. Same spelling `bridge-zenoh-to-cyclonedds-fwd`
/// and `tests/ros2_pub.cpp` use, and the one Cyclone's descriptor table is
/// keyed on for `create_publisher`.
const TYPE_NAME: &str = "std_msgs::msg::dds_::String_";

/// The ROS-form key the runtime registry uses. NUL-terminated: the pointer
/// goes straight into Cyclone's C descriptor table.
const REG_TYPE_NAME: &str = "std_msgs/msg/String\0";

/// `std_msgs/msg/String`, as a schema Cyclone can build a descriptor from.
static STRING_FIELDS: &[Field] = &[Field {
    name: "data\0",
    ty: FieldType::String,
    offset: 0,
}];

/// The payload the probe publishes. A peer that decodes it byte-equal is the
/// peer AGREEING with `get_serialization_format`; a peer that receives bytes
/// it cannot decode would not print this.
const PAYLOAD: &str = "advertised-state-probe";

fn env_u64(key: &str, default: u64) -> u64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(default)
}

fn env_str(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// One QoS profile as `key=value` pairs on a single line.
///
/// Every field, in the ABI's own units, with no interpretation: the test
/// compares NUMBERS against the peer's rendered profile and against the
/// requested profile, and a probe that pretty-printed `RELIABLE` here would be
/// doing half of that comparison itself.
fn fmt_qos(q: &rmw_qos_profile_t) -> String {
    format!(
        "reliability={} durability={} history={} depth={} deadline_ms={} lifespan_ms={} \
         liveliness_kind={} liveliness_lease_ms={}",
        q.reliability,
        q.durability,
        q.history,
        q.depth,
        q.deadline_ms,
        q.lifespan_ms,
        q.liveliness_kind,
        q.liveliness_lease_ms
    )
}

fn hex24(bytes: &[u8; 24]) -> String {
    let mut s = String::with_capacity(48);
    for b in bytes {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

/// CDR-LE encoding of `std_msgs::msg::dds_::String_ { data }`.
///
/// Encapsulation header (`00 01 00 00`), then the string length INCLUDING its
/// NUL, then the bytes and the NUL. Same layout as `tests/ros2_pub.cpp`, which
/// a stock `ros2 topic echo` already decodes.
fn cdr_string(data: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(8 + data.len() + 1);
    out.extend_from_slice(&[0x00, 0x01, 0x00, 0x00]);
    let n = (data.len() + 1) as u32;
    out.extend_from_slice(&n.to_le_bytes());
    out.extend_from_slice(data.as_bytes());
    out.push(0);
    out
}

fn main() {
    let domain: u32 = env_u64("ROS_DOMAIN_ID", 0) as u32;
    let pub_topic = env_str("ADV_PROBE_PUB_TOPIC", "nros_advertised_pub");
    let sub_topic = env_str("ADV_PROBE_SUB_TOPIC", "nros_advertised_sub");
    let node_name = env_str("ADV_PROBE_NODE", "advertised_state_probe");
    let hold = Duration::from_millis(env_u64("ADV_PROBE_HOLD_MS", 90_000));
    let tick = Duration::from_millis(env_u64("ADV_PROBE_TICK_MS", 200).max(10));

    println!("ADV_PROBE_DOMAIN {domain}");

    if nros_rmw_cyclonedds_sys::register().is_err() {
        eprintln!("ADV_PROBE_FAIL: nros_rmw_cyclonedds_sys::register() failed");
        std::process::exit(2);
    }
    // Cyclone refuses a topic whose type has no registered descriptor
    // (`publisher.cpp` returns UNSUPPORTED from `find_descriptor`), and the
    // registrar is installed by the register above — so the order is
    // load-bearing, not stylistic.
    if let Err(e) = nros_rmw::register_type_descriptor(REG_TYPE_NAME, STRING_FIELDS) {
        eprintln!("ADV_PROBE_FAIL: register_type_descriptor: {e:?}");
        std::process::exit(2);
    }

    let backend = CString::new("cyclonedds").expect("literal");
    // SAFETY: `backend` is a live NUL-terminated string for the call.
    let vt: *const NrosRmwVtable = unsafe { nros_rmw_cffi_lookup(backend.as_ptr()) };
    if vt.is_null() {
        eprintln!("ADV_PROBE_FAIL: no vtable registered under `cyclonedds`");
        std::process::exit(2);
    }
    // SAFETY: the registry hands out a `'static` vtable; see `Registry`'s
    // publication contract in nros-rmw-cffi.
    let vt: &'static NrosRmwVtable = unsafe { &*vt };

    // The four slots under test, named individually. A NULL here is the
    // interesting failure — it is what zenoh, XRCE and uORB all look like —
    // so it exits with its own code and says which slot rather than
    // panicking somewhere downstream.
    let mut missing: Vec<&str> = Vec::new();
    if vt.publisher_count_matched_subscriptions.is_none() {
        missing.push("publisher_count_matched_subscriptions");
    }
    if vt.subscription_count_matched_publishers.is_none() {
        missing.push("subscription_count_matched_publishers");
    }
    if vt.get_gid_for_publisher.is_none() {
        missing.push("get_gid_for_publisher");
    }
    if vt.publisher_get_actual_qos.is_none() {
        missing.push("publisher_get_actual_qos");
    }
    if vt.subscription_get_actual_qos.is_none() {
        missing.push("subscription_get_actual_qos");
    }
    if vt.get_serialization_format.is_none() {
        missing.push("get_serialization_format");
    }
    if !missing.is_empty() {
        for m in &missing {
            eprintln!("ADV_SLOT_NULL {m}");
        }
        eprintln!(
            "ADV_PROBE_FAIL: {} advertised-state slot(s) are NULL",
            missing.len()
        );
        std::process::exit(3);
    }
    println!("ADV_PROBE_SLOTS_OK");

    let node_c = CString::new(node_name.clone()).expect("node name");
    let ns_c = CString::new("/").expect("literal");
    let pub_c = CString::new(pub_topic.clone()).expect("pub topic");
    let sub_c = CString::new(sub_topic.clone()).expect("sub topic");
    let type_c = CString::new(TYPE_NAME).expect("literal");
    let hash_c = CString::new("").expect("literal");

    let mut session = rmw_session_t {
        node_name: node_c.as_ptr(),
        namespace_: ns_c.as_ptr(),
        _reserved: [0u8; 8],
        backend_data: core::ptr::null_mut(),
    };
    // SAFETY: every pointer above outlives the session; `out` is ours.
    let rc = unsafe {
        (vt.create_session.expect("create_session"))(
            core::ptr::null(),
            0,
            domain,
            node_c.as_ptr(),
            core::ptr::null(),
            &mut session,
        )
    };
    if rc != NROS_RMW_RET_OK {
        eprintln!("ADV_PROBE_FAIL: create_session -> {rc}");
        std::process::exit(4);
    }

    let mut node = rmw_node_t {
        name: node_c.as_ptr(),
        namespace_: ns_c.as_ptr(),
        session: &mut session,
        _reserved: [0u8; 8],
        backend_data: core::ptr::null_mut(),
    };
    // `create_node` is one of the four `default` slots — a backend that does
    // not fill it leaves the runtime to simply remember the node, so a NULL is
    // not a failure. Cyclone fills it (that is what publishes
    // `ros_discovery_info`, which is how the peer learns our node NAME), and
    // without it `ros2 topic info --verbose` would report our endpoints under
    // an empty node name and the per-node endpoint selection this test needs
    // could not work.
    if let Some(create_node) = vt.create_node {
        // SAFETY: `session` outlives `node`; both are ours.
        let rc = unsafe { create_node(&mut session, node_c.as_ptr(), ns_c.as_ptr(), &mut node) };
        if rc != NROS_RMW_RET_OK {
            eprintln!("ADV_PROBE_FAIL: create_node -> {rc}");
            std::process::exit(4);
        }
    }

    // The REQUESTED profile: zero-filled, i.e. SYSTEM_DEFAULT everywhere. See
    // the module doc — this is what makes the read-back falsifiable.
    let requested = rmw_qos_profile_t {
        reliability: 0,
        durability: 0,
        history: 0,
        liveliness_kind: 0,
        depth: 0,
        _reserved0: 0,
        deadline_ms: 0,
        lifespan_ms: 0,
        liveliness_lease_ms: 0,
        avoid_ros_namespace_conventions: 0,
        _reserved1: [0u8; 3],
    };
    let ts = rmw_message_type_support_t {
        type_name: type_c.as_ptr(),
        type_hash: hash_c.as_ptr(),
    };

    let mut publisher = rmw_publisher_t {
        topic_name: pub_c.as_ptr(),
        type_name: type_c.as_ptr(),
        qos: requested,
        can_loan_messages: false,
        _reserved: [0u8; 7],
        backend_data: core::ptr::null_mut(),
    };
    // SAFETY: node, type support and topic name all outlive the publisher.
    let rc = unsafe {
        (vt.create_publisher.expect("create_publisher"))(
            &node,
            &ts,
            pub_c.as_ptr(),
            domain,
            &requested,
            core::ptr::null(),
            &mut publisher,
        )
    };
    if rc != NROS_RMW_RET_OK {
        eprintln!("ADV_PROBE_FAIL: create_publisher({pub_topic}) -> {rc}");
        std::process::exit(4);
    }

    let mut subscription = rmw_subscription_t {
        topic_name: sub_c.as_ptr(),
        type_name: type_c.as_ptr(),
        qos: requested,
        can_loan_messages: false,
        _reserved: [0u8; 7],
        backend_data: core::ptr::null_mut(),
    };
    // SAFETY: as above.
    let rc = unsafe {
        (vt.create_subscription.expect("create_subscription"))(
            &node,
            &ts,
            sub_c.as_ptr(),
            domain,
            &requested,
            core::ptr::null(),
            &mut subscription,
        )
    };
    if rc != NROS_RMW_RET_OK {
        eprintln!("ADV_PROBE_FAIL: create_subscription({sub_topic}) -> {rc}");
        std::process::exit(4);
    }

    // ---- the four families, read once, printed verbatim -----------------

    // SAFETY: the slot takes no arguments and returns a `'static` literal.
    let fmt_ptr = unsafe { (vt.get_serialization_format.expect("slot"))() };
    let format = if fmt_ptr.is_null() {
        // The dispatcher in `nros-rmw-cffi` maps a NULL slot to "the backend
        // has not said", never to a guessed "cdr". A NULL RETURN from a
        // non-NULL slot is the same claim and gets the same treatment.
        "(none)".to_string()
    } else {
        // SAFETY: a non-NULL return from this slot is a NUL-terminated
        // `'static` string (`vtable.cpp` returns a literal).
        unsafe { core::ffi::CStr::from_ptr(fmt_ptr) }
            .to_string_lossy()
            .into_owned()
    };
    println!("ADV_FORMAT {format}");

    let mut gid = rmw_gid_t {
        implementation_identifier: core::ptr::null(),
        // Not zero: a slot that wrote nothing would then be indistinguishable
        // from one that reported an all-zero gid, and "all-zero" is the ABI's
        // spelling for "unknown".
        data: [0xAA; 24],
    };
    // SAFETY: the publisher was created above; `gid` is ours.
    let rc = unsafe { (vt.get_gid_for_publisher.expect("slot"))(&publisher, &mut gid) };
    if rc != NROS_RMW_RET_OK {
        eprintln!("ADV_PROBE_FAIL: get_gid_for_publisher -> {rc}");
        std::process::exit(5);
    }
    println!("ADV_GID {}", hex24(&gid.data));

    println!("ADV_QOS_REQUESTED {}", fmt_qos(&requested));

    // The ABI's contract is that `out` ARRIVES carrying the request and a
    // field the backend cannot report is left as it came in, so pre-loading it
    // is not a shortcut — it is how the slot is specified to be called
    // (`rmw_vtable.h`, and `CffiSession::create_publisher` does the same).
    let mut pub_granted = requested;
    // SAFETY: as above.
    let rc = unsafe { (vt.publisher_get_actual_qos.expect("slot"))(&publisher, &mut pub_granted) };
    if rc != NROS_RMW_RET_OK {
        eprintln!("ADV_PROBE_FAIL: publisher_get_actual_qos -> {rc}");
        std::process::exit(5);
    }
    println!("ADV_QOS_PUB_GRANTED {}", fmt_qos(&pub_granted));

    let mut sub_granted = requested;
    // SAFETY: the subscription was created above; `sub_granted` is ours.
    let rc =
        unsafe { (vt.subscription_get_actual_qos.expect("slot"))(&subscription, &mut sub_granted) };
    if rc != NROS_RMW_RET_OK {
        eprintln!("ADV_PROBE_FAIL: subscription_get_actual_qos -> {rc}");
        std::process::exit(5);
    }
    println!("ADV_QOS_SUB_GRANTED {}", fmt_qos(&sub_granted));

    println!("ADV_PROBE_READY node={node_name} pub={pub_topic} sub={sub_topic}");

    // ---- the matched-count state machine --------------------------------
    //
    // POLLED, not sampled once: these slots report what discovery has ALREADY
    // delivered and never block, so a single call after a peer starts
    // legitimately returns the answer from before it did. Same Design note as
    // `graph_interop`'s.
    //
    // The EDGES are what a peer can prove and a self-test cannot. A stub
    // returning a constant, or the local entity count, satisfies "the slot
    // returns OK" and satisfies a single sample; it cannot follow a peer
    // appearing and then leaving.
    let payload = cdr_string(PAYLOAD);
    let span = rmw_byte_span_t {
        data: payload.as_ptr(),
        len: payload.len(),
    };

    let deadline = Instant::now() + hold;
    let mut last = (usize::MAX, usize::MAX);
    let (mut pub_rose, mut pub_fell) = (false, false);
    let (mut sub_rose, mut sub_fell) = (false, false);

    while Instant::now() < deadline {
        // SAFETY: publisher live, payload live for the call.
        let _ = unsafe { (vt.publish.expect("publish"))(&publisher, span) };

        let mut np = 0usize;
        let mut ns = 0usize;
        // SAFETY: entities live; the out-params are ours.
        let rp = unsafe {
            (vt.publisher_count_matched_subscriptions.expect("slot"))(&publisher, &mut np)
        };
        // SAFETY: as above.
        let rs = unsafe {
            (vt.subscription_count_matched_publishers.expect("slot"))(&subscription, &mut ns)
        };
        if rp != NROS_RMW_RET_OK || rs != NROS_RMW_RET_OK {
            eprintln!("ADV_PROBE_FAIL: matched-count slot returned pub={rp} sub={rs}");
            std::process::exit(5);
        }

        if (np, ns) != last {
            println!("ADV_MATCHED pub={np} sub={ns}");
            last = (np, ns);
        }
        // Each edge is announced ONCE, in order: a rise that has not happened
        // cannot be followed by a fall, so a probe that printed FELL from the
        // initial zero would report an edge nobody caused.
        if np >= 1 && !pub_rose {
            pub_rose = true;
            println!("ADV_PUB_MATCHED_ROSE {np}");
        }
        if np == 0 && pub_rose && !pub_fell {
            pub_fell = true;
            println!("ADV_PUB_MATCHED_FELL");
        }
        if ns >= 1 && !sub_rose {
            sub_rose = true;
            println!("ADV_SUB_MATCHED_ROSE {ns}");
        }
        if ns == 0 && sub_rose && !sub_fell {
            sub_fell = true;
            println!("ADV_SUB_MATCHED_FELL");
        }

        std::thread::sleep(tick);
    }

    println!(
        "ADV_PROBE_DONE pub_rose={pub_rose} pub_fell={pub_fell} sub_rose={sub_rose} sub_fell={sub_fell}"
    );

    // SAFETY: each entity was created by the matching slot above and is
    // destroyed exactly once, subscription and publisher before the session.
    unsafe {
        let _ = (vt.destroy_subscription.expect("slot"))(&mut subscription);
        let _ = (vt.destroy_publisher.expect("slot"))(&mut publisher);
        if let Some(destroy_node) = vt.destroy_node {
            let _ = destroy_node(&mut node);
        }
        let _ = (vt.destroy_session.expect("slot"))(&mut session);
    }
}
