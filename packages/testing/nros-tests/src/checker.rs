//! RFC-0051 / phase-295 W2 — the ONE standard-node output checker.
//!
//! Every example node follows the stock ROS 2 demo behavior contract
//! (`talker`/`listener`, `add_two_ints`, Fibonacci action), so ONE checker
//! asserts them all — replacing the ~55 inline copies of the same
//! wait-ready → collect → count → assert dance the 2026-07-17 survey found
//! across the per-cell e2e files, and the 22 hardcoded `"Received:"`
//! literals (every marker string lives in [`crate::output`]; a grep gate
//! keeps it that way — audit E7).
//!
//! The checker is TRANSPORT- and PEER-agnostic: it reads process output.
//! The same functions assert a nano-ros listener, a `ros2 topic echo`
//! stream, or a `demo_nodes_cpp` peer in the interop lanes — behavioral
//! interchangeability with the ROS 2 demos is the contract being pinned.

use crate::{matrix::Workload, output};

/// Which side of the pair a process plays.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Role {
    /// talker / service server / action server — the side that must be
    /// READY before the peer starts.
    Producer,
    /// listener / service client / action client — the side whose output
    /// proves delivery.
    Consumer,
}

/// The ready marker to wait for before starting the peer (`None` = the
/// role has no ready contract for this workload — start immediately).
pub fn ready_marker(workload: Workload, role: Role) -> Option<&'static str> {
    match (workload, role) {
        (Workload::Pubsub, Role::Producer) => Some(output::TALKER_READY_MARKER),
        (Workload::Service, Role::Producer) => Some(output::SERVICE_SERVER_READY_MARKER),
        (Workload::Action, Role::Producer) => Some(output::ACTION_SERVER_READY_MARKER),
        // Listeners print nothing until traffic arrives; clients drive.
        _ => None,
    }
}

/// The delivery marker whose occurrences prove the workload ran.
pub fn delivery_marker(workload: Workload) -> &'static str {
    match workload {
        Workload::Pubsub | Workload::EntryPubsub | Workload::CustomMsg | Workload::Multihost => {
            output::LISTENER_LOG_PREFIX
        }
        // Remap: the external int32-sink on the REMAPPED topic proves delivery.
        Workload::RealtimeTiers | Workload::Qos | Workload::Remap => {
            output::INT32_LISTENER_LOG_PREFIX
        }
        Workload::Service => output::SERVICE_RESULT_PREFIX,
        Workload::Action => output::ACTION_RESULT_PREFIX,
        // Logging / params / lifecycle / safety lanes assert their own
        // marker sets from `output`; delivery here means "the demo's
        // final line appeared".
        Workload::Logging => output::LISTENER_LOG_PREFIX,
        Workload::Params | Workload::Lifecycle | Workload::Safety => output::LISTENER_LOG_PREFIX,
        // Not delivery at all — the fixture's own verdict line. `assert_delivery`
        // counting it once is exactly the contract.
        Workload::Errno => output::ERRNO_ISOLATION_PASS,
        // Also a verdict rather than delivery: a discovery workload proves it
        // SAW the peer, which no message count can express.
        Workload::Graph => output::GRAPH_PROBE_SAW,
        // Also a verdict: the evidence is that a status event reached an
        // application callback at all, which no message count can express.
        Workload::QosEvents => output::QOS_EVENT_LIVELINESS_CHANGED,
    }
}

/// Assert the consumer-side output proves the workload delivered.
///
/// Panics with the cell-diagnosable message shape (`expected ≥N …` +
/// the full output) every current per-cell file hand-rolls. For pubsub
/// workloads this ALSO checks payload monotonicity via the `output`
/// parsers (the stock-demo counter contract).
pub fn assert_delivery(workload: Workload, out: &str, min_events: usize) {
    match workload {
        Workload::Pubsub | Workload::EntryPubsub => {
            output::assert_listener(out, min_events);
        }
        Workload::Service => {
            let n = crate::count_pattern(out, output::SERVICE_RESULT_PREFIX);
            assert!(
                n >= min_events,
                "expected ≥{min_events} `{}` lines, got {n}.\nOutput:\n{out}",
                output::SERVICE_RESULT_PREFIX
            );
        }
        Workload::Action => {
            output::assert_action_client(out);
        }
        _ => {
            let marker = delivery_marker(workload);
            let n = crate::count_pattern(out, marker);
            assert!(
                n >= min_events,
                "expected ≥{min_events} `{marker}` lines, got {n}.\nOutput:\n{out}",
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pubsub_delivery_asserts_monotonic() {
        let out = format!(
            "{}\n{}\n{}\n",
            crate::output::listener_line(1),
            crate::output::listener_line(2),
            crate::output::listener_line(3)
        );
        assert_delivery(Workload::Pubsub, &out, 3);
    }

    #[test]
    #[should_panic]
    fn pubsub_delivery_fails_short() {
        let out = crate::output::listener_line(1);
        assert_delivery(Workload::Pubsub, &out, 2);
    }

    #[test]
    fn service_delivery_counts_results() {
        let out = format!("{}\n", crate::output::service_result_line(5));
        assert_delivery(Workload::Service, &out, 1);
    }

    #[test]
    fn ready_markers_only_for_producers() {
        assert!(ready_marker(Workload::Pubsub, Role::Producer).is_some());
        assert!(ready_marker(Workload::Pubsub, Role::Consumer).is_none());
        assert!(ready_marker(Workload::Action, Role::Producer).is_some());
    }
}
