//! phase-433 W6 acceptance fixture — make a QoS status event FIRE, live.
//!
//! Four vtable slots (`publisher_event_init`, `publisher_take_event`,
//! `subscription_event_init`, `subscription_take_event`) are classified
//! `produced` by `just check rmw-slot-producers` and had never been run against
//! a live peer. `produced` is not evidence: phase-381 shipped twelve graph
//! slots that were produced, mutation-tested and parity-clean while the feature
//! did not work at all (issue 0903), because every check compared our code to
//! our own builders.
//!
//! A QoS event is the extreme case of that, because most of the family is
//! STRUCTURALLY unable to fire in one image:
//!
//! * `LivelinessLost` and `OfferedDeadlineMissed` are publisher-side
//!   SELF-observations — the zenoh shim compares its own `last_publish_at_ms` /
//!   `last_assert_at_ms` against its own clock. No peer takes part, so running
//!   them against one proves nothing new.
//! * `RequestedDeadlineMissed` is the same clock comparison one entity over.
//! * `MessageLost` needs a peer to produce a sequence GAP, which no test can
//!   induce on demand.
//! * `LivelinessChanged` is the only kind whose input is a REMOTE entity's
//!   state: it fires when the set of publishers holding a `@ros2_lv` liveliness
//!   token matching this subscription's topic changes. A single image has no
//!   remote publisher to change, and — the part that makes it an INTEROP claim
//!   rather than a two-nano-node claim — the token whose keyexpr we have to
//!   match is written by `rmw_zenoh_cpp`, not by us. The tree's only existing
//!   check of that wildcard (`shim/mod.rs`, `publisher_keyexpr_wildcard`)
//!   matches it against a keyexpr OUR OWN builder produced.
//!
//! So this probe registers `on_liveliness_changed` and reports what it sees.
//!
//! ## What it does NOT do
//!
//! It does not decode a message, and it does not care whether one arrives.
//! Delivery is covered by `ros2-string-interop` on the same topic; conflating
//! the two would let a delivery success mask an event that never fired.
//!
//! ## Exit codes — an empty observation is a FAILURE, not a quiet pass
//!
//! * `0` — at least one `LivelinessChanged` with a POSITIVE `alive_count_change`
//!   was delivered. Positive specifically: the peer appearing is the transition
//!   we can arrange, and a zero-delta callback would be the backend reporting
//!   "nothing changed", which is what a broken poll also reports.
//! * `2` — the subscription could not be created.
//! * `3` — the backend declined the event kind (`supports_event` false, or
//!   `on_liveliness_changed` returned an error). This is a legitimate,
//!   INFORMATIVE answer for a backend whose `subscription_event_init` slot is
//!   NULL, and it is deliberately distinct from "registered but never fired".
//! * `4` — registered, spun the whole budget, observed nothing.
//!
//! ## Environment
//!
//! * `NROS_LOCATOR`                 zenoh locator (default `tcp/127.0.0.1:7447`)
//! * `ROS_DOMAIN_ID`                domain (default 0) — the first chunk of the
//!                                  liveliness keyexpr, so a mismatch here is
//!                                  indistinguishable from a missing peer
//!                                  (issue 0927, the same trap in `graph-probe`)
//! * `QOS_EVENT_PROBE_TOPIC`        topic (default `/chatter`)
//! * `QOS_EVENT_PROBE_TIMEOUT_MS`   total budget (default 25000)

use core::time::Duration;
use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use log::{error, info};
use nros::{Executor, ExecutorConfig};

/// The stock `demo_nodes_cpp` talker's type, in the DDS spelling the keyexpr
/// carries. Same constants as `ros2-string-interop`, which subscribes to the
/// same peer on the same topic for the DELIVERY half of this pairing.
const TYPE_NAME: &str = "std_msgs::msg::dds_::String_";
const TYPE_HASH: &str = "TypeHashNotSupported";

/// One observed `LivelinessChanged`, flattened to primitives.
///
/// `nros_rmw::LivelinessChangedStatus` would do, but the callback bound is
/// `FnMut(..) + Send + 'static`, so the sink is shared and the tuple keeps the
/// lock's payload obviously `Send` without a second thought.
type Observed = (u16, u16, i16, i16);

fn env_u64(key: &str, default: u64) -> u64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    nros_rmw_zenoh::register().expect("register zenoh backend");

    let locator = std::env::var("NROS_LOCATOR").unwrap_or_else(|_| "tcp/127.0.0.1:7447".into());
    let topic = std::env::var("QOS_EVENT_PROBE_TOPIC").unwrap_or_else(|_| "/chatter".into());
    let budget_ms = env_u64("QOS_EVENT_PROBE_TIMEOUT_MS", 25_000);
    // The domain is the first chunk of `@ros2_lv/<domain>/...`, so getting it
    // wrong produces "no matching token" — which reads exactly like "the event
    // does not work". `graph-probe` was filed as a broken Cyclone reader for
    // this reason (issue 0927); do not let the constructor default it.
    let domain_id: u32 = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(0);

    let cfg = ExecutorConfig::new(&locator)
        .node_name("qos_event_probe")
        .namespace("/")
        .domain_id(domain_id);
    let mut exec = Executor::open_with_rmw("zenoh", &cfg).expect("open zenoh session");

    let mut sub = {
        let mut node = exec.create_node("qos_event_probe").expect("create node");
        match node.create_subscription_raw(&topic, TYPE_NAME, TYPE_HASH) {
            Ok(s) => s,
            Err(e) => {
                error!("subscription create failed: {e:?}");
                println!("QOS_EVENT_PROBE_SUB_FAILED err={e:?}");
                std::process::exit(2);
            }
        }
    };

    // Report the CAPABILITY before the outcome. A backend that says "no" here
    // and one that says "yes" and never fires are different defects, and the
    // test asserts on which of the two it got.
    let supports_liveliness = sub.supports_event(nros_rmw::EventKind::LivelinessChanged);
    println!(
        "QOS_EVENT_SUPPORTS LivelinessChanged={} RequestedDeadlineMissed={} MessageLost={}",
        supports_liveliness,
        sub.supports_event(nros_rmw::EventKind::RequestedDeadlineMissed),
        sub.supports_event(nros_rmw::EventKind::MessageLost),
    );

    let sink: Arc<Mutex<Vec<Observed>>> = Arc::new(Mutex::new(Vec::new()));
    let cb_sink = Arc::clone(&sink);
    if let Err(e) = sub.on_liveliness_changed(move |s| {
        if let Ok(mut v) = cb_sink.lock() {
            v.push((
                s.alive_count,
                s.not_alive_count,
                s.alive_count_change,
                s.not_alive_count_change,
            ));
        }
    }) {
        // The honest answer for a backend whose `subscription_event_init` slot
        // is NULL. Not a skip and not a pass: the caller asked for a capability
        // and was refused, which is a fact worth exiting distinctly on.
        println!("QOS_EVENT_PROBE_UNSUPPORTED kind=LivelinessChanged err={e:?}");
        std::process::exit(3);
    }

    println!(
        "QOS_EVENT_PROBE_READY locator={locator} domain={domain_id} topic={topic} budget_ms={budget_ms}"
    );

    let deadline = Instant::now() + Duration::from_millis(budget_ms);
    let mut reported = 0usize;
    let mut saw_positive = false;

    while Instant::now() < deadline {
        let _ = exec.spin_once(Duration::from_millis(100));
        // The liveliness poll is driven from the subscription's `has_data`, not
        // from the executor's spin: this subscription was created directly, so
        // nothing in the dispatch loop touches it and a probe that only spun
        // would poll nothing at all. Drain any payload too, so a full RX buffer
        // can never be the reason `has_data` stops being called.
        let _ = sub.has_data();
        while let Ok(Some(_)) = sub.take_serialized() {}

        // Drain outside the callback so the print is not inside the backend's
        // poll path.
        let drained: Vec<Observed> = match sink.lock() {
            Ok(mut v) => v.drain(..).collect(),
            Err(_) => Vec::new(),
        };
        for (alive, not_alive, alive_change, not_alive_change) in drained {
            println!(
                "QOS_EVENT LIVELINESS_CHANGED alive={alive} not_alive={not_alive} \
                 alive_change={alive_change} not_alive_change={not_alive_change}"
            );
            reported += 1;
            if alive_change > 0 {
                saw_positive = true;
            }
        }
        if saw_positive {
            // The transition we can arrange has happened. Keep going briefly is
            // NOT what we want: the peer teardown delta is the test's business,
            // not the probe's, and a probe that lingers turns a pass into a
            // timeout under load.
            break;
        }
    }

    println!("QOS_EVENT_PROBE_COUNT {reported}");
    if saw_positive {
        info!("liveliness event observed");
        return;
    }
    // Say which of the two nothings this was. "Registered, fired zero times" is
    // the finding phase-433 W6 exists to be able to state; "fired, but never
    // with a positive delta" would mean the poll runs and reports no change.
    println!(
        "QOS_EVENT_PROBE_NONE registered=true supports={supports_liveliness} \
         events_seen={reported} budget_ms={budget_ms}"
    );
    std::process::exit(4);
}
