//! phase-296 W5.1 — `SystemModel → MapperInput` derivation (RFC-0052
//! §"nano-ros execution modeling").
//!
//! nano-ros does its OWN causality/execution modeling: it derives the mapper
//! INPUT from the model's **input** layers (structure + contracts) and feeds
//! the shared platform-agnostic core
//! ([`ros_launch_manifest_sched::chain_aware_rank`]) — it does NOT consume any
//! resolved sched plan (there is none; `execution.sched` was reverted, rlm
//! `f090400`). The core returns a priorityless
//! [`ros_launch_manifest_sched::RankedPlan`]; the RTOS realizer (W5.2) turns
//! that into kernel-feature scheduling.
//!
//! v1 scope: the **node side** — `MapperNode` (scope, criticality, per-path
//! trigger/latency facts) from `structure.nodes` + `contracts.node_paths` +
//! `contracts.pub_endpoints`. Chains are **empty** (the model carries no chain
//! declarations today), so the core degrades to criticality-bucketed RM/DM —
//! the design's graceful-degradation path. Chain-declaration input is a later
//! wave.

use ros_launch_manifest_model::SystemModel;
use ros_launch_manifest_sched::{
    MapperInput, MapperNode, RankedPlan,
    chain::{EffectiveTrigger, MapperPath},
    chain_aware_rank,
    mapper::Criticality,
};

/// Parse the model's advisory criticality string (`high`|`medium`|`low`) into
/// the mapper's [`Criticality`]. Unknown/absent → `None` (the mapper treats a
/// node with no criticality as the lowest bucket).
fn parse_criticality(s: &str) -> Option<Criticality> {
    match s.trim().to_ascii_lowercase().as_str() {
        "high" => Some(Criticality::High),
        "medium" | "med" => Some(Criticality::Medium),
        "low" => Some(Criticality::Low),
        _ => None,
    }
}

/// The declared publisher rate (Hz) for an endpoint ref, if the model carries a
/// `min_rate_hz` contract for it. Used as the fire rate of a periodic (timer)
/// path — a path with no `input` fires on a clock, and its output endpoint's
/// contracted rate is the honest "how often" fact.
use crate::wcet::WcetProfile;

fn pub_rate_hz(model: &SystemModel, endpoint_ref: &str) -> Option<f64> {
    model
        .contracts
        .pub_endpoints
        .get(endpoint_ref)
        .and_then(|c| c.min_rate_hz)
}

/// Derive one node's causal paths from `contracts.node_paths`. A node path is
/// keyed `"<node_fqn>/<path_name>"`; a path belongs to `fqn` when its key has
/// that prefix and the remainder is a single segment (the path name).
fn node_paths_for(model: &SystemModel, fqn: &str, wcet: Option<&WcetProfile>) -> Vec<MapperPath> {
    let prefix = format!("{fqn}/");
    let mut out = Vec::new();
    for (path_ref, pc) in &model.contracts.node_paths {
        let Some(name) = path_ref.strip_prefix(&prefix) else {
            continue;
        };
        // Guard against a longer node FQN prefix-matching a shorter one: the
        // path name is a single segment (no further `/`).
        if name.contains('/') {
            continue;
        }
        // Trigger: an empty `input` is a timer/periodic path (fires on a
        // clock at the output's contracted rate); otherwise it is event-driven
        // on its input endpoints.
        let effective_trigger = if pc.input.is_empty() {
            let rate_hz = pc
                .output
                .iter()
                .find_map(|o| pub_rate_hz(model, o))
                .unwrap_or(0.0);
            EffectiveTrigger::Timer { rate_hz }
        } else {
            EffectiveTrigger::Input(pc.input.clone())
        };
        out.push(MapperPath {
            name: name.to_string(),
            effective_trigger,
            max_latency_ms: pc.max_latency_ms,
            // RFC-0078 — the ONLY source of a WCET is a declaration, looked up
            // at rlm's own boundary identity (`path_ref` is `node_fqn/path`,
            // which is exactly what `boundaries_without_wcet` reports). `None`
            // when undeclared, when the profile carries no clock rate, or when
            // there is no profile at all — and `None` is not zero: rlm counts
            // the boundary as undeclared and says so. Never invent one.
            exec_ms: wcet.and_then(|w| w.exec_ms(path_ref)),
            inputs: pc.input.clone(),
            outputs: pc.output.clone(),
            // play_launch phase 68 W5 / nano-ros phase 434 — both crossed the
            // model boundary so THIS side could read them, and until now it
            // did not: rlm's `MapperPath` carried the bound, the model carried
            // it, and the copy built here filled in `None`. Read straight
            // through; `None` still means undeclared, and only when the
            // contract declared nothing.
            max_jitter_ms: pc.max_jitter_ms,
            miss: pc.miss.clone(),
        });
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    out
}

/// Derive a [`MapperInput`] from the SystemModel's input layers. Nodes come
/// from `structure.nodes` + their `contracts.node_paths`; chains are empty in
/// v1 (the model carries no chain declarations).
pub fn mapper_input_from_model(model: &SystemModel) -> MapperInput {
    mapper_input_from_model_with_wcet(model, None)
}

/// As [`mapper_input_from_model`], with a selected RFC-0078 measurement profile
/// supplying `exec_ms` for the boundaries it declares.
///
/// Separate entry point rather than a changed signature so that ABSENT remains
/// the default at the API level too: a caller that knows nothing about WCETs
/// gets exactly what it got before, which is `None` everywhere. Which profile
/// is selected is deliberately not decided here — RFC-0078 leaves that to
/// `system.toml` / SystemModel, and this function takes the answer.
pub fn mapper_input_from_model_with_wcet(
    model: &SystemModel,
    wcet: Option<&WcetProfile>,
) -> MapperInput {
    let mut nodes = Vec::with_capacity(model.structure.nodes.len());
    for (fqn, node) in &model.structure.nodes {
        let criticality = node.criticality.as_deref().and_then(parse_criticality);
        // The exclusion relation (play_launch phase 67/68). PRESENCE in
        // `node_concurrency` is the declaration: no entry means every path
        // of the node serialises — what rclcpp's implicit callback group
        // does, and what `default_cbg_type` here does too — while an entry
        // is the author claiming MORE concurrency than that. That claim is
        // what makes a per-thread reservation unsound and a summed chain
        // latency optimistic, so it has to reach the mapper. Until phase 434
        // it stopped at the model, and the two toolchains scheduled the same
        // system from opposite defaults.
        let claims_concurrency = model.contracts.node_concurrency.get(fqn).is_some_and(|c| {
            c.exclusive.iter().flatten().count() < node_paths_for(model, fqn, None).len()
        });
        nodes.push(MapperNode {
            name: fqn.clone(),
            scope: node.scope.clone(),
            criticality,
            paths: node_paths_for(model, fqn, wcet),
            claims_concurrency,
            ..Default::default()
        });
    }
    MapperInput {
        nodes,
        legacy: None,
        chains: Vec::new(),
    }
}

/// Convenience: derive the input and run the shared platform-agnostic core,
/// returning the priorityless [`RankedPlan`] the RTOS realizer (W5.2) consumes.
pub fn rank_from_model(model: &SystemModel) -> RankedPlan {
    chain_aware_rank(&mapper_input_from_model(model))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_launch_manifest_model::{
        Contracts, Execution, NodeInstance, PathContract, PubContract, Structure, SystemModel,
        TopicContract,
    };
    use std::collections::BTreeMap;

    /// RFC-0078 — a declared profile reaches `MapperPath::exec_ms`, and nothing
    /// else does.
    ///
    /// The fixture's boundaries are `/sensor/acquire` and `/planner/plan`, which
    /// is the real key shape: a ROS node FQN begins with `/` and may carry
    /// namespaces. Writing this test is what caught a validator rule that
    /// required exactly two segments and would have rejected every boundary in
    /// the tree.
    #[test]
    fn a_declared_wcet_reaches_exec_ms_and_an_undeclared_one_stays_absent() {
        use crate::wcet::{BoundaryWcet, WcetProfile};

        let model = model_with_two_nodes();
        let profile = WcetProfile {
            cpu: "cortex-m4f".into(),
            clock_hz: Some(1_000_000),
            profile: "release".into(),
            measured_at_commit: "a1b2c3d4e5f6".into(),
            counter_valid: true,
            source: "nros.wcet.measurements/1".into(),
            // A margin is what turns the high-water mark into a bound. Without
            // it this profile would declare an observation and yield NO
            // exec_ms — which is the behaviour `an_observation_alone_reaches_
            // nothing` below pins.
            margin_percent: Some(0.0),
            coverage: Some("fixture: fixed inputs".into()),
            boundaries: BTreeMap::from([(
                "/sensor/acquire".to_string(),
                BoundaryWcet {
                    min_observed_cycles: 1_000,
                    max_observed_cycles: 2_500,
                    iterations: 100,
                    bound_cycles: None,
                },
            )]),
        };
        assert!(profile.validate().is_empty(), "{:?}", profile.validate());

        let input = mapper_input_from_model_with_wcet(&model, Some(&profile));
        let exec = |node: &str, path: &str| {
            input
                .nodes
                .iter()
                .find(|n| n.name == node)
                .and_then(|n| n.paths.iter().find(|p| p.name == path))
                .and_then(|p| p.exec_ms)
        };

        // 2_500 cycles at 1 MHz = 2.5 ms.
        let declared = exec("/sensor", "acquire").expect("declared boundary must carry exec_ms");
        assert!((declared - 2.5).abs() < 1e-9, "got {declared}");

        // The other boundary was not declared. Absent, not zero — rlm will count
        // it as undeclared and say so via ChainFeasibleWithoutWcet.
        assert_eq!(
            exec("/planner", "plan"),
            None,
            "an undeclared boundary must stay absent; a zero here would claim \
             measured headroom nobody has"
        );
    }

    /// An observation with no bound reaches `exec_ms` as `None`, even though
    /// the boundary IS declared and the rate IS known.
    ///
    /// This is the research finding wired end to end: a maximum observed over N
    /// runs is a high-water mark, and letting it through would hand rlm an
    /// under-estimate that looks measured.
    #[test]
    fn an_observation_alone_reaches_nothing() {
        use crate::wcet::{BoundaryWcet, WcetProfile};

        let model = model_with_two_nodes();
        let profile = WcetProfile {
            cpu: "cortex-m4f".into(),
            clock_hz: Some(1_000_000),
            profile: "release".into(),
            measured_at_commit: "a1b2c3d4e5f6".into(),
            counter_valid: true,
            source: "nros.wcet.measurements/1".into(),
            margin_percent: None, // no bound derived from the observation
            coverage: None,
            boundaries: BTreeMap::from([(
                "/sensor/acquire".to_string(),
                BoundaryWcet {
                    min_observed_cycles: 1_000,
                    max_observed_cycles: 2_500,
                    iterations: 100,
                    bound_cycles: None,
                },
            )]),
        };
        assert!(
            profile.validate().is_empty(),
            "the declaration is well-formed"
        );

        let input = mapper_input_from_model_with_wcet(&model, Some(&profile));
        assert!(
            input
                .nodes
                .iter()
                .flat_map(|n| &n.paths)
                .all(|p| p.exec_ms.is_none()),
            "an observation without a declared bound must not reach exec_ms"
        );
    }

    /// Without a profile the derivation is byte-for-byte what it was before
    /// RFC-0078 — absent is the default at the API level, not just in the data.
    #[test]
    fn no_profile_means_no_exec_ms_anywhere() {
        let model = model_with_two_nodes();
        let input = mapper_input_from_model(&model);
        assert!(
            input
                .nodes
                .iter()
                .flat_map(|n| &n.paths)
                .all(|p| p.exec_ms.is_none()),
            "the no-profile path must invent nothing"
        );
    }

    fn model_with_two_nodes() -> SystemModel {
        let mut nodes = indexmap::IndexMap::new();
        nodes.insert(
            "/sensor".to_string(),
            NodeInstance {
                scope: "/".to_string(),
                criticality: Some("high".to_string()),
                ..Default::default()
            },
        );
        nodes.insert(
            "/planner".to_string(),
            NodeInstance {
                scope: "/".to_string(),
                criticality: Some("medium".to_string()),
                ..Default::default()
            },
        );

        let mut pub_endpoints = BTreeMap::new();
        pub_endpoints.insert(
            "/sensor/scan".to_string(),
            PubContract {
                min_rate_hz: Some(20.0),
                ..Default::default()
            },
        );

        let mut node_paths = BTreeMap::new();
        // /sensor: periodic (empty input) publishing /sensor/scan at 20 Hz.
        node_paths.insert(
            "/sensor/acquire".to_string(),
            PathContract {
                input: vec![],
                output: vec!["/sensor/scan".to_string()],
                max_latency_ms: Some(5.0),
                ..Default::default()
            },
        );
        // /planner: event-driven on /planner/objects_in → /planner/cmd.
        node_paths.insert(
            "/planner/plan".to_string(),
            PathContract {
                input: vec!["/planner/objects_in".to_string()],
                output: vec!["/planner/cmd".to_string()],
                max_latency_ms: Some(30.0),
                ..Default::default()
            },
        );

        SystemModel {
            structure: Structure {
                nodes,
                ..Default::default()
            },
            contracts: Contracts {
                pub_endpoints,
                node_paths,
                topics: BTreeMap::<String, TopicContract>::new(),
                ..Default::default()
            },
            execution: Execution::default(),
            ..Default::default()
        }
    }

    /// Phase 434 — the three fields play_launch carried across the model
    /// boundary for this crate reach the mapper. Before this they stopped at
    /// the model: `max_jitter_ms`/`miss` were filled `None` here, and the
    /// concurrency relation was never read at all, so an author's claim that
    /// two callbacks may run at once — which is what makes a summed chain
    /// latency optimistic and a per-thread reservation unsound — was invisible
    /// on this side.
    #[test]
    fn contract_seams_reach_the_mapper() {
        use ros_launch_manifest_sched::{ConcurrencyContract, MapperMiss};
        let mut model = model_with_two_nodes();
        model
            .contracts
            .node_paths
            .get_mut("/planner/plan")
            .unwrap()
            .max_jitter_ms = Some(4.0);
        model
            .contracts
            .node_paths
            .get_mut("/planner/plan")
            .unwrap()
            .miss = Some(MapperMiss {
            tolerate_n: Some(1),
            ..Default::default()
        });
        // /planner has two paths; declaring `exclusive: [[plan]]` leaves the
        // other free to run concurrently — a claim of MORE concurrency.
        model.contracts.node_paths.insert(
            "/planner/telemetry".to_string(),
            PathContract {
                input: vec![],
                output: vec!["/planner/stats".to_string()],
                ..Default::default()
            },
        );
        model.contracts.node_concurrency.insert(
            "/planner".to_string(),
            ConcurrencyContract {
                exclusive: vec![vec!["plan".to_string()]],
            },
        );

        let input = mapper_input_from_model(&model);
        let planner = input.nodes.iter().find(|n| n.name == "/planner").unwrap();
        let plan = planner.paths.iter().find(|p| p.name == "plan").unwrap();
        assert_eq!(plan.max_jitter_ms, Some(4.0));
        assert_eq!(plan.miss.as_ref().and_then(|m| m.tolerate_n), Some(1));
        assert!(
            planner.claims_concurrency,
            "a declared exclusion that leaves a path out claims concurrency"
        );
        let sensor = input.nodes.iter().find(|n| n.name == "/sensor").unwrap();
        assert!(
            !sensor.claims_concurrency,
            "no declaration means everything serialises"
        );
    }

    #[test]
    fn derives_nodes_paths_and_triggers() {
        let model = model_with_two_nodes();
        let input = mapper_input_from_model(&model);

        assert_eq!(input.nodes.len(), 2);
        assert!(input.chains.is_empty(), "v1 carries no chains");

        let sensor = input.nodes.iter().find(|n| n.name == "/sensor").unwrap();
        assert_eq!(sensor.criticality, Some(Criticality::High));
        assert_eq!(sensor.paths.len(), 1);
        // Empty-input path → Timer at the output's contracted 20 Hz.
        assert_eq!(
            sensor.paths[0].effective_trigger,
            EffectiveTrigger::Timer { rate_hz: 20.0 }
        );

        let planner = input.nodes.iter().find(|n| n.name == "/planner").unwrap();
        assert_eq!(planner.criticality, Some(Criticality::Medium));
        // Non-empty input → event-driven.
        assert_eq!(
            planner.paths[0].effective_trigger,
            EffectiveTrigger::Input(vec!["/planner/objects_in".to_string()])
        );
        assert_eq!(planner.paths[0].max_latency_ms, Some(30.0));
    }

    #[test]
    fn feeds_the_agnostic_core() {
        // The derived input runs through the shared core (no chains → the
        // criticality-bucketed RM/DM degradation path). Proves the
        // SystemModel → MapperInput → chain_aware_rank pipeline end to end.
        let model = model_with_two_nodes();
        let ranked = rank_from_model(&model);
        // Both nodes' paths are ranked; the high-criticality sensor outranks
        // the medium planner.
        assert!(!ranked.items.is_empty());
        let sensor_pos = ranked.items.iter().position(|i| i.node == "/sensor");
        let planner_pos = ranked.items.iter().position(|i| i.node == "/planner");
        assert!(sensor_pos.is_some() && planner_pos.is_some());
        assert!(
            sensor_pos < planner_pos,
            "High-criticality /sensor must outrank medium /planner: {:?}",
            ranked.items
        );
    }
}
