//! phase-392 W5.b/W5.c — the entity figures a backend must size its tables
//! from, resolved from the declaration and printed as env lines.
//!
//! Sibling of [`crate::cmd::board_facts`], and deliberately shaped like it: one
//! resolution, one implementation, delivered through the process environment
//! because that is the only carrier that reaches the cargo invocation a
//! workspace member is built by (issue 0460, phase-349 W2.0).
//!
//! **What it answers.** `ZPICO_MAX_QUERYABLES` sizes `SERVICE_BUFFERS` (as
//! `ZPICO_MAX_SESSIONS * ZPICO_MAX_QUERYABLES`) and the C shim's per-session
//! queryable table. Its default was `if hosted { 32 } else { 8 }` — a literal
//! picked for headroom in `nros-zpico-build`, because nothing at that point
//! knows the answer. It cost a native talker 144,128 B of service buffers for
//! services it does not have. Two halves decide the real number and this verb
//! carries both:
//!
//! * `NROS_DECLARED_SERVICE_SERVERS` — the APPLICATION's own count, from the
//!   model's `structure.services` / `structure.actions` wiring. An action
//!   server is three services on the wire, so actions multiply.
//! * `NROS_DECLARED_INFRA_QUERYABLES` — whether the ROS parameter services and
//!   the REP-2002 lifecycle services are in the image, from
//!   `execution.features`. This is the half a build script cannot see any other
//!   way: cargo exposes no other crate's features.
//!
//! The consumer adds the infrastructure COST (`PARAM_SERVICE_QUERYABLES`,
//! `LIFECYCLE_SERVICE_QUERYABLES`, both defined beside the code that creates
//! them). This verb never states those numbers — it says which features are on,
//! not what they cost, which is the split that keeps issue 0460 closed.
//!
//! **Why the model and not a knob.** The launch declaration is the contract
//! (phase-392 W5.b2). A floor-plus-headroom rule would re-create in miniature
//! the guess this wave exists to delete, and a guess derived from something
//! real is still a guess nobody can audit.

use std::{collections::BTreeMap, path::PathBuf};

use clap::Args as ClapArgs;
use eyre::{Result, WrapErr, bail};
use ros_launch_manifest_model::SystemModel;

/// One action server is three zenoh queryables (`send_goal`, `cancel_goal`,
/// `get_result`; feedback and status are topics).
///
/// MIRROR of `nros_node::executor::action::ACTION_SERVER_QUERYABLES`, held to
/// its definition by `check-infra-queryable-counts`. The CLI cannot depend on
/// `nros-node` — that crate is `no_std`, platform-gated and built for the
/// target, not the host — so the number is restated here and gated, which is
/// the whole difference between this and the seven prose spellings issue 0827
/// found.
const ACTION_SERVER_QUERYABLES: usize = 3;

#[derive(Debug, ClapArgs)]
pub struct EntityFactsArgs {
    /// Resolved SystemModel to read. Mutually exclusive with `--bringup-dir`.
    #[arg(long, value_name = "PATH")]
    pub model: Option<PathBuf>,

    /// The bringup package DIRECTORY, addressed the way `nano_ros_entry(LAUNCH
    /// …)` and `nros::main!(launch = …)` address it. The model path is derived
    /// by `nros_orchestration_ir::model_location`, never spelled here.
    #[arg(long = "bringup-dir", value_name = "DIR", conflicts_with = "model")]
    pub bringup_dir: Option<PathBuf>,

    /// Launch file relative to `<bringup>/launch/`. Defaults to the bringup's
    /// `[system] default_launch`.
    #[arg(long, value_name = "FILE", requires = "bringup_dir")]
    pub launch: Option<String>,

    /// Launch argument binding `key=value` (repeatable), for an arg-bound
    /// model variant.
    #[arg(long = "arg", value_name = "K=V", requires = "bringup_dir")]
    pub args: Vec<String>,
}

/// The two facts, in emission order.
///
/// A pure function over the model, with file IO and the environment lifted out:
/// a sizing rule verified by reading is how this campaign's other defects
/// survived (phase-392 W5.d).
pub fn facts_from_model(model: &SystemModel) -> BTreeMap<String, String> {
    let mut out = BTreeMap::new();
    if let Some(n) = declared_service_servers(model) {
        out.insert("NROS_DECLARED_SERVICE_SERVERS".to_string(), n.to_string());
    }
    out.insert(
        "NROS_DECLARED_INFRA_QUERYABLES".to_string(),
        declared_infra(model).to_string(),
    );
    out
}

/// Whether this model DESCRIBES the graph's wiring at all.
///
/// **Abstaining here is the DESIGNED outcome, not a symptom (issue 0973).**
/// Endpoint wiring is AUTHORED, never derived. A plain `<node>` launch file
/// names a node; it does not say what that node publishes or serves, and
/// nothing else in the resolver's inputs does either. `model_builder` fills
/// `structure.{topics,services,actions}` from a `ManifestIndex`, and
/// `manifest_loader` builds that index from a CONTRACT — the provider sidecar
///
/// ```text
/// <bringup>/launch/<stem>.contract.yaml     beside <stem>.launch.xml
/// ```
///
/// or an overlay root passed as `--contracts <dir>`. That file is the ONE input
/// a user authors to make this function true, and naming it is the difference
/// between "the model does not say" being actionable and being merely true.
///
/// So an absent `services` map does NOT mean "this system has no service
/// servers" — it means nobody stated the answer. Reporting 0 there would size
/// the queryable table to the infrastructure alone and exhaust it the moment a
/// node registers: a confident wrong number, sized exactly, which is the
/// failure shape this campaign keeps finding rather than a new one. The
/// discriminator is whether ANY wiring was described. If it was, an empty
/// `services` map is a real zero. If nothing was, the question is unanswered
/// and this verb says nothing rather than guessing.
///
/// A caller must therefore not read the abstain as "the resolver lost
/// something". Nothing is lost; the input does not exist. (There WAS one real
/// instance of loss — the loader silently dropped `actions:` — and R1-P2 fixed
/// it. Check for a contract file before searching the resolver again.)
///
/// **Re-measured 2026-09-06, and the count moves — the correspondence does
/// not.** Resolving every launch file in the tree: 122 `*.launch.xml`, 5
/// `*.contract.yaml`, and of the 114 that resolve standalone exactly 5 describe
/// wiring — the same 5, in `examples/workspaces/cpp/src/demo_bringup/launch/`.
/// It read `0 of 119` when issue 0973 was answered on 2026-09-03 and changed
/// the day phase-412 landed the first contracts. Quote the invariant (wiring
/// <=> an authored contract), never the number.
///
/// A consumer wanting per-image entity counts writes a contract; there is no
/// second source. The per-component `nano_ros_node_register(... ENTITIES ...)`
/// route issue 0900 took is RETIRED (phase-412) — it is now a `FATAL_ERROR`
/// naming this same file, because a list hand-maintained beside the code
/// drifted from it on the safety island and every derived pool came out short.
///
/// Sibling predicate, and it is NOT identical: `EntityInventory::from_model`
/// also accepts a non-empty `contracts.node_paths`, which is where a contract's
/// timer paths land. A timer-only contract is therefore wiring to that
/// consumer and silence to this one — issue 1140.
fn describes_wiring(model: &SystemModel) -> bool {
    !model.structure.topics.is_empty()
        || !model.structure.services.is_empty()
        || !model.structure.actions.is_empty()
}

/// Every service server the model declares, counted as QUERYABLES.
///
/// `ServiceWiring::server` lists the endpoint refs serving a service
/// (`"<node FQN>/<endpoint>"`), so the count is the number of refs, not the
/// number of services: two nodes serving the same name are two queryables.
///
/// Actions are the same wiring type in a separate map and cost
/// [`ACTION_SERVER_QUERYABLES`] each.
///
/// The whole model is counted rather than one node's share: an entry image
/// realizes its model, and the queryable table is per SESSION, which the image
/// has one of.
fn declared_service_servers(model: &SystemModel) -> Option<usize> {
    if !describes_wiring(model) {
        return None;
    }
    let services: usize = model
        .structure
        .services
        .values()
        .map(|s| s.server.len())
        .sum();
    let actions: usize = model
        .structure
        .actions
        .values()
        .map(|a| a.server.len())
        .sum();
    Some(services + actions * ACTION_SERVER_QUERYABLES)
}

/// Which infrastructure service families the image carries.
///
/// UNKNOWN feature names are ignored on purpose. `execution.features` is an
/// open axis — `safety` is in the tree today and is not a queryable question —
/// so a name this verb does not recognise means "not one of mine", never an
/// error. The consumer refuses an unknown SPELLING of this verb's own output,
/// which is a different thing: that would be a broken channel, not an
/// unrelated feature.
fn declared_infra(model: &SystemModel) -> &'static str {
    let has = |name: &str| model.execution.features.iter().any(|f| f == name);
    match (has("param_services"), has("lifecycle")) {
        (true, true) => "param+lifecycle",
        (true, false) => "param",
        (false, true) => "lifecycle",
        (false, false) => "none",
    }
}

pub fn run(args: EntityFactsArgs) -> Result<()> {
    let model_path = match (&args.model, &args.bringup_dir) {
        (Some(m), _) => m.clone(),
        (None, Some(b)) => {
            let bringup = b
                .canonicalize()
                .wrap_err_with(|| format!("bringup dir `{}`", b.display()))?;
            let mut launch_args: Vec<(String, String)> = Vec::new();
            for kv in &args.args {
                let Some((k, v)) = kv.split_once('=') else {
                    bail!("--arg takes `key=value`, got `{kv}`");
                };
                launch_args.push((k.to_string(), v.to_string()));
            }
            let rel = nros_orchestration_ir::model_location::launch_to_model_rel(
                &bringup,
                args.launch.as_deref(),
                &launch_args,
            )
            .map_err(|e| eyre::eyre!(e))?;
            nros_orchestration_ir::model_location::resolve_model_path(&bringup, &rel)
        }
        (None, None) => bail!("entity-facts needs --model <path> or --bringup-dir <dir>"),
    };

    let model = crate::orchestration::model_ingest::load_model(&model_path)?;
    for (k, v) in facts_from_model(&model) {
        println!("{k}={v}");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model(yaml: &str) -> SystemModel {
        SystemModel::from_yaml_str(yaml).expect("test model parses")
    }

    const EMPTY: &str = "meta:\n  version: 1\nstructure: {}\n";

    #[test]
    fn a_model_that_describes_no_wiring_abstains_on_the_app_count() {
        // NOT zero. 109 of the tree's 114 resolvable models are this shape
        // (measured 2026-09-06), including `examples/workspaces/c`'s, whose
        // node is literally called `add_server` and whose model says nothing
        // about services because that workspace authors no contract.
        let m = model(EMPTY);
        assert_eq!(declared_service_servers(&m), None);
        assert_eq!(declared_infra(&m), "none");
        let f = facts_from_model(&m);
        assert!(!f.contains_key("NROS_DECLARED_SERVICE_SERVERS"));
        // The infrastructure half is still answered — that is W5.b1, and it is
        // the half a build script cannot see any other way.
        assert_eq!(f["NROS_DECLARED_INFRA_QUERYABLES"], "none");
    }

    #[test]
    fn wiring_described_with_no_service_server_is_a_real_zero() {
        // A model that describes topics has been through a resolver that
        // describes wiring, so an empty `services` map means what it says.
        let m = model(
            "meta:\n  version: 1\nstructure:\n  topics:\n    /chatter:\n      type: std_msgs/msg/String\n\
             \n      pub: [\"/a/chatter\"]\n",
        );
        assert_eq!(declared_service_servers(&m), Some(0));
        assert_eq!(facts_from_model(&m)["NROS_DECLARED_SERVICE_SERVERS"], "0");
    }

    #[test]
    fn service_servers_are_counted_per_endpoint_not_per_service() {
        // Two nodes serving the same service name are two queryables. Counting
        // the map's keys would say one and under-size the table.
        let m = model(
            "meta:\n  version: 1\nstructure:\n  services:\n    /add:\n      type: example/srv/Add\n\
             \n      server: [\"/a/add\", \"/b/add\"]\n",
        );
        assert_eq!(declared_service_servers(&m), Some(2));
    }

    #[test]
    fn a_client_only_service_costs_no_queryable() {
        let m = model(
            "meta:\n  version: 1\nstructure:\n  services:\n    /add:\n      type: example/srv/Add\n\
             \n      client: [\"/a/add\"]\n",
        );
        assert_eq!(declared_service_servers(&m), Some(0));
    }

    #[test]
    fn an_action_server_costs_three() {
        let m = model(
            "meta:\n  version: 1\nstructure:\n  actions:\n    /fib:\n      type: example/action/Fib\n\
             \n      server: [\"/a/fib\"]\n",
        );
        assert_eq!(declared_service_servers(&m), Some(ACTION_SERVER_QUERYABLES));
    }

    #[test]
    fn services_and_actions_add() {
        let m = model(
            "meta:\n  version: 1\nstructure:\n  services:\n    /add:\n      type: example/srv/Add\n\
             \n      server: [\"/a/add\"]\n  actions:\n    /fib:\n      type: example/action/Fib\n\
             \n      server: [\"/a/fib\"]\n",
        );
        assert_eq!(
            declared_service_servers(&m),
            Some(1 + ACTION_SERVER_QUERYABLES)
        );
    }

    #[test]
    fn infra_reads_both_flags_independently() {
        let f = |feats: &str| {
            let m = model(&format!(
                "meta:\n  version: 1\nstructure: {{}}\nexecution:\n  features:\n{feats}"
            ));
            declared_infra(&m)
        };
        assert_eq!(f("  - param_services\n  - lifecycle\n"), "param+lifecycle");
        assert_eq!(f("  - param_services\n"), "param");
        assert_eq!(f("  - lifecycle\n"), "lifecycle");
    }

    #[test]
    fn an_unrecognised_feature_is_not_an_error_and_not_a_queryable() {
        // `safety` is a real feature in this tree and has nothing to do with
        // queryables. A verb that refused it would break every safety
        // workspace to answer a question it was not asked.
        let m = model("meta:\n  version: 1\nstructure: {}\nexecution:\n  features:\n  - safety\n");
        assert_eq!(declared_infra(&m), "none");
        let m = model(
            "meta:\n  version: 1\nstructure: {}\nexecution:\n  features:\n  - safety\n  - lifecycle\n",
        );
        assert_eq!(declared_infra(&m), "lifecycle");
    }

    #[test]
    fn the_emitted_shape_is_exactly_what_the_consumer_reads() {
        // `nros-zpico-build::queryable_default_from` reads these two names and
        // PANICS on a spelling it does not know, so the pair is a contract.
        let m = model(
            "meta:\n  version: 1\nstructure:\n  services:\n    /add:\n      type: example/srv/Add\n\
             \n      server: [\"/a/add\"]\nexecution:\n  features:\n  - lifecycle\n",
        );
        let f = facts_from_model(&m);
        assert_eq!(f["NROS_DECLARED_SERVICE_SERVERS"], "1");
        assert_eq!(f["NROS_DECLARED_INFRA_QUERYABLES"], "lifecycle");
        assert_eq!(f.len(), 2);
    }
}
