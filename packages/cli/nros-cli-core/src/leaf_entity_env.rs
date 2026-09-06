//! Issue 0827 — a cargo leaf's pool budgets, derived from what it declares.
//!
//! The pools that dominate static RAM (`ZPICO_MAX_SUBSCRIBERS`,
//! `ZPICO_MAX_QUERYABLES`, `ZPICO_MAX_PUBLISHERS`) are sized in
//! `nros-rmw-zenoh`'s build script. That crate is a DEPENDENCY of the leaf,
//! and the entities are declared in the leaf's own `src/lib.rs`, so the crate
//! that must know the counts compiles BEFORE the crate whose source states
//! them. No build script, proc macro or manifest key can reach backwards
//! across that edge.
//!
//! It does not have to. The probe already ran: `nros sync` writes
//! `<leaf>/metadata/<component>.json` describing what the component creates.
//! This module turns that file into the same [`EntityDecl`] rows a
//! `nano_ros_node_register(... ENTITIES ...)` would have produced, so the
//! COUNTING RULES stay in [`crate::entity_inventory`] where CMake consumers
//! already get them. A second derivation would be a second answer.
//!
//! Both ends are per-host artifacts and neither is committed: the probe output
//! is gitignored (`examples/**/metadata/*.json`), and so is the `[env]` sidecar
//! this renders. A fresh clone has neither and gets both from `nros sync` — the
//! contract `generated/` already has.
//!
//! A leaf whose metadata is `<component>.json.unprobeable` cannot be probed at
//! all — the probe compiles the component for the HOST, and neither a foreign
//! `[build] target` with `[unstable] build-std` nor a board crate with no host
//! build allows that. Issue 1061.
//!
//! For those, the leaf DECLARES instead, in its own manifest:
//!
//! ```toml
//! [package.metadata.nros.component]
//! entities = ["publisher:std_msgs/msg/String:/chatter", "timer"]
//! ```
//!
//! Same grammar as `nano_ros_node_register(... ENTITIES ...)`, parsed by the
//! same [`EntityDecl::parse`], because a second spelling of one declaration is
//! how the two drift. Nothing is compiled to read it.
//!
//! **A declaration is CROSS-CHECKED against the probe wherever the probe can
//! run.** That is what keeps a hand-written list from quietly going stale: on a
//! probeable leaf the two must agree, and a mismatch REFUSES rather than
//! picking one. The declaration exists for leaves where there is nothing to
//! check it against; it does not become a way to override what the code says.

use std::{collections::BTreeMap, path::Path};

use eyre::{Result, WrapErr};
use serde::Deserialize;

use crate::entity_inventory::{
    ComponentEntities, Declaration, EntityDecl, EntityInventory, EntityKind,
};

/// The probe's per-node lists, and the [`EntityKind`] each one means.
///
/// One table, so a kind cannot be silently dropped by being handled in one
/// place and forgotten in another. `guard_condition` has no probe key — the
/// Rust node metadata does not describe one — and its absence here is
/// deliberate rather than an oversight.
const NODE_ENTITY_KEYS: &[(&str, EntityKind)] = &[
    ("publishers", EntityKind::Publisher),
    ("subscribers", EntityKind::Subscription),
    ("timers", EntityKind::Timer),
    ("services", EntityKind::ServiceServer),
    ("service_clients", EntityKind::ServiceClient),
    ("actions", EntityKind::ActionServer),
    ("action_clients", EntityKind::ActionClient),
];

#[derive(Debug, Deserialize)]
struct ProbeInterface {
    package: Option<String>,
    name: Option<String>,
}

#[derive(Debug, Deserialize)]
struct ProbeEntity {
    id: Option<String>,
    interface: Option<ProbeInterface>,
}

#[derive(Debug, Deserialize)]
struct ProbeNode {
    #[serde(flatten)]
    lists: BTreeMap<String, serde_json::Value>,
}

#[derive(Debug, Deserialize)]
struct ProbeDoc {
    #[serde(default)]
    package: Option<String>,
    #[serde(default)]
    component: Option<String>,
    #[serde(default)]
    nodes: Vec<ProbeNode>,
}

/// `example_interfaces` + `action/Fibonacci` -> `example_interfaces/action/Fibonacci`.
fn qualified_type(iface: &ProbeInterface) -> Option<String> {
    match (iface.package.as_deref(), iface.name.as_deref()) {
        (Some(p), Some(n)) => Some(format!("{p}/{n}")),
        _ => None,
    }
}

/// Turn ONE probe document into the rows the inventory counts.
pub fn declaration_from_probe(doc_json: &str) -> Result<(String, String, Declaration)> {
    let doc: ProbeDoc =
        serde_json::from_str(doc_json).wrap_err("leaf metadata is not the probe's JSON shape")?;
    let pkg = doc.package.clone().unwrap_or_else(|| "<unknown>".into());
    let comp = doc.component.clone().unwrap_or_else(|| "<unknown>".into());

    let mut decls: Vec<EntityDecl> = Vec::new();
    for node in &doc.nodes {
        for (key, kind) in NODE_ENTITY_KEYS {
            let Some(v) = node.lists.get(*key) else {
                continue;
            };
            let Some(arr) = v.as_array() else { continue };
            for item in arr {
                let ent: ProbeEntity = match serde_json::from_value(item.clone()) {
                    Ok(e) => e,
                    // A row this module cannot read is NOT skipped quietly: it
                    // would lower a pool below what the image creates, and short
                    // halts the board. Refuse the whole leaf instead.
                    Err(e) => {
                        return Err(eyre::eyre!(
                            "leaf metadata has a `{key}` entry this module cannot read ({e}); \
                             refusing rather than deriving a budget that is short"
                        ));
                    }
                };
                decls.push(EntityDecl {
                    kind: *kind,
                    type_name: ent.interface.as_ref().and_then(qualified_type),
                    name: ent.id,
                    depth: None,
                });
            }
        }
    }

    // `Stated(vec![])` and `None` are the same COUNT and different FACTS: the
    // probe ran and found nothing, versus a component that never declared. The
    // probe running IS a statement, so an empty result is `None` (asserts it
    // creates nothing), never `Absent`.
    let declaration = if decls.is_empty() {
        Declaration::None
    } else {
        Declaration::Stated(decls)
    };
    Ok((pkg, comp, declaration))
}

/// Issue 1061 — the entities a leaf DECLARES in its own manifest.
///
/// `[package.metadata.nros.component] entities = [...]`, each string in the
/// `nano_ros_node_register(... ENTITIES ...)` grammar. `Ok(None)` means the key
/// is absent, which is different from an empty list: an empty list is a leaf
/// asserting it creates nothing.
pub fn declared_entities(leaf: &Path) -> Result<Option<Vec<EntityDecl>>> {
    let manifest = leaf.join("Cargo.toml");
    let Ok(text) = std::fs::read_to_string(&manifest) else {
        return Ok(None);
    };
    let doc: toml::Value =
        toml::from_str(&text).wrap_err_with(|| format!("parsing {}", manifest.display()))?;
    let Some(v) = doc
        .get("package")
        .and_then(|p| p.get("metadata"))
        .and_then(|m| m.get("nros"))
        .and_then(|n| n.get("component"))
        .and_then(|c| c.get("entities"))
    else {
        return Ok(None);
    };
    let arr = v.as_array().ok_or_else(|| {
        eyre::eyre!(
            "{}: `[package.metadata.nros.component] entities` must be an ARRAY of \
             declaration strings, e.g. [\"publisher:std_msgs/msg/String:/chatter\", \"timer\"]",
            manifest.display()
        )
    })?;
    let mut out = Vec::new();
    for item in arr {
        let spec = item.as_str().ok_or_else(|| {
            eyre::eyre!(
                "{}: every `entities` element must be a string; found {item}",
                manifest.display()
            )
        })?;
        // The SAME parser the CMake path uses. A private grammar here is how a
        // declaration means one thing in a CMakeLists and another in a manifest.
        let decls = EntityDecl::parse(spec)
            .map_err(|e| eyre::eyre!("{}: entities entry `{spec}`: {e}", manifest.display()))?;
        out.extend(decls);
    }
    Ok(Some(out))
}

/// Compare a declaration with what the probe found, as multisets of KIND.
///
/// Kind only, deliberately. The probe resolves topic names and interfaces that
/// a hand-written declaration may legitimately state loosely (`timer` carries
/// neither), so comparing the full row would refuse honest declarations. What a
/// budget is computed from is the per-kind COUNT, so that is what has to agree —
/// a mismatch there is a mismatch in the numbers this module exists to produce.
fn kind_counts(decls: &[EntityDecl]) -> BTreeMap<&'static str, usize> {
    let mut m = BTreeMap::new();
    for d in decls {
        *m.entry(d.kind.tag()).or_insert(0) += 1;
    }
    m
}

/// `Err` when a declaration and a successful probe disagree.
pub fn reconcile(component: &str, declared: &[EntityDecl], probed: &[EntityDecl]) -> Result<()> {
    let (d, p) = (kind_counts(declared), kind_counts(probed));
    if d == p {
        return Ok(());
    }
    let fmt = |m: &BTreeMap<&'static str, usize>| {
        if m.is_empty() {
            "nothing".to_string()
        } else {
            m.iter()
                .map(|(k, v)| format!("{k}x{v}"))
                .collect::<Vec<_>>()
                .join(", ")
        }
    };
    Err(eyre::eyre!(
        "{component}: the manifest declares {} but the code creates {}.\n  \
         Refusing rather than choosing one: a budget from the declaration would be \
         wrong for the image, and silently preferring the probe would let the \
         declaration rot until it reaches a leaf where nothing can check it.\n  \
         Fix the `[package.metadata.nros.component] entities` list, or drop it and \
         let the probe answer.",
        fmt(&d),
        fmt(&p)
    ))
}

/// Every probeable component under `<leaf>/metadata/`, as one image's inventory.
///
/// `.json.unprobeable` files are skipped BY NAME and reported, because their
/// existence is the reason a leaf may get no sidecar at all — a silent skip
/// would render a budget for half an image.
pub fn inventory_for_leaf(leaf: &Path) -> Result<(EntityInventory, Vec<String>)> {
    let dir = leaf.join("metadata");
    let mut inv = EntityInventory::new(dir.display().to_string());
    let mut unprobeable = Vec::new();
    // Kept beside the inventory so the reconcile below can read what was probed
    // without `EntityInventory` growing an accessor for one caller.
    let mut probed_rows: Vec<(String, Vec<EntityDecl>)> = Vec::new();
    // Issue 1061 — what the leaf DECLARES, if anything. Read before the probe
    // results so it can serve both roles: the answer where nothing was probed,
    // and the cross-check where something was.
    let declared = declared_entities(leaf)?;

    let Ok(rd) = std::fs::read_dir(&dir) else {
        // No `metadata/` at all. A declaration still answers — that is the whole
        // point for a leaf the probe cannot reach.
        if let Some(d) = declared {
            inv.insert(ComponentEntities {
                pkg: leaf_name(leaf),
                component: leaf_name(leaf),
                class: leaf_name(leaf),
                declaration: if d.is_empty() {
                    Declaration::None
                } else {
                    Declaration::Stated(d)
                },
            });
        }
        return Ok((inv, unprobeable));
    };
    let mut entries: Vec<_> = rd.filter_map(|e| e.ok()).map(|e| e.path()).collect();
    entries.sort();
    for path in entries {
        let name = path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or_default();
        if name.ends_with(".json.unprobeable") {
            unprobeable.push(name.to_string());
            continue;
        }
        if !name.ends_with(".json") {
            continue;
        }
        let text = std::fs::read_to_string(&path)
            .wrap_err_with(|| format!("reading {}", path.display()))?;
        let (pkg, component, declaration) = declaration_from_probe(&text)
            .wrap_err_with(|| format!("parsing {}", path.display()))?;
        probed_rows.push((component.clone(), declaration.entities().to_vec()));
        inv.insert(ComponentEntities {
            pkg,
            component: component.clone(),
            class: component,
            declaration,
        });
    }
    // Issue 1061 — reconcile, or stand in.
    match (&declared, inv.is_empty()) {
        // Probed AND declared: they must agree. Checked per component only when
        // there is exactly one, because a multi-component leaf's manifest states
        // one list for the whole package and splitting it across components
        // would be inventing an attribution.
        (Some(d), false) => {
            if let [(component, probed)] = probed_rows.as_slice() {
                reconcile(component, d, probed)?;
            }
        }
        // Declared with nothing probed: the declaration IS the answer. This is
        // the unprobeable leaf, which is what issue 1061 is about.
        (Some(d), true) => {
            inv.insert(ComponentEntities {
                pkg: leaf_name(leaf),
                component: leaf_name(leaf),
                class: leaf_name(leaf),
                declaration: if d.is_empty() {
                    Declaration::None
                } else {
                    Declaration::Stated(d.clone())
                },
            });
        }
        (None, _) => {}
    }
    Ok((inv, unprobeable))
}

/// The leaf directory's own name, used as pkg/component when a declaration
/// stands in for a probe that never ran and there is no probed name to use.
fn leaf_name(leaf: &Path) -> String {
    leaf.file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("<leaf>")
        .to_string()
}

/// The knobs a derived budget sets, and which field answers each.
///
/// Names are the ones the BUILD SCRIPTS read, checked against them rather than
/// guessed: `ZPICO_*` are `zpico-sys`/`nros-zpico-build`, `NROS_RMW_*` is
/// `nros-rmw-cffi`, `NROS_EXECUTOR_*` is `nros-node`.
pub const DERIVED_ENV_KEYS: &[&str] = &[
    "NROS_EXECUTOR_ACTION_CLIENTS",
    "NROS_EXECUTOR_MAX_CBS",
    "NROS_RMW_SUBSCRIBER_SLOTS",
    "ZPICO_MAX_PUBLISHERS",
    "ZPICO_MAX_SUBSCRIBERS",
];

/// Issue 1125 — the PAYLOAD-CLASS keys, which come from a second inventory.
///
/// Kept apart from [`DERIVED_ENV_KEYS`] because they empty independently and
/// for different reasons: the entity budget needs only the probe, while these
/// need the message-bound artifacts too and REFUSE when a subscribed type
/// carries no bound. A leaf can legitimately get one group and not the other,
/// so a single list would have to be "sometimes present", which is what makes
/// an absent key unreadable.
///
/// `ZPICO_SUBSCRIBER_LARGE_SIZE` is here and is emitted only when the large
/// count is non-zero — with zero blocks the pool is zero bytes whatever the
/// size says, and naming a size for a class that does not exist would be
/// inventing a number. Same rule as `_nros_bounds_publish_payload_classes`.
pub const DERIVED_PAYLOAD_ENV_KEYS: &[&str] = &[
    "NROS_SUBSCRIBER_BUFFER_SIZE",
    "ZPICO_MAX_LARGE_SUBSCRIBERS",
    "ZPICO_SUBSCRIBER_LARGE_SIZE",
];

/// `ZPICO_MAX_QUERYABLES` is DELIBERATELY NOT DERIVED.
///
/// `DerivedEntityKnobs::max_queryables` says so itself: it counts service
/// servers and actions and "does NOT include the parameter or lifecycle service
/// families (`PARAM_SERVICE_QUERYABLES` 6, `LIFECYCLE_SERVICE_QUERYABLES` 5):
/// those are per-image infrastructure enabled by a feature this inventory
/// cannot see". Its own conclusion is that "an image carrying them must still
/// state the knob".
///
/// The CMake path completes it with `NROS_DECLARED_INFRA_QUERYABLES`. A cargo
/// leaf has no such channel, so a sidecar emitting the bare count would state a
/// number that is SHORT for any image with param services — and a short
/// queryable table is not a smaller pool, it is a registration failure at boot.
/// The crate default (8) is larger and safe, so the knob is left alone.
///
/// Found by measuring rather than by reading: the esp32 talker hand-sets it to
/// 2, the derivation offered 1, and only the leaf's own `[env]` winning kept
/// the image correct. A leaf that had not hand-set it would have taken the 1.
const NOT_DERIVED_NEEDS_INFRA_COUNT: &str = "ZPICO_MAX_QUERYABLES";

/// Render the gitignored `[env]` sidecar for a derived budget.
pub fn render_env_sidecar(
    knobs: &crate::entity_inventory::DerivedEntityKnobs,
    payload: &crate::leaf_payload_classes::PayloadClasses,
    source: &str,
) -> String {
    let mut s = String::new();
    s.push_str("# GENERATED by `nros sync` (issue 0827) — DO NOT EDIT.\n#\n");
    s.push_str(
        "# Pool budgets derived from what this leaf's components DECLARE, read\n\
         # from the metadata probe's output. The counting rules are\n\
         # `nros_cli_core::entity_inventory` — the same ones a CMake image gets,\n\
         # so a cargo leaf and a configured image cannot disagree.\n#\n",
    );
    s.push_str(
        "# NOT committed, and it must not be: it is derived from a probe output\n\
         # that is itself per-host and gitignored. A fresh clone regenerates both\n\
         # with `nros sync`.\n#\n",
    );
    s.push_str(&format!("# source: {source}\n"));
    s.push_str(
        "#\n# An environment value the caller sets WINS over this file: cargo's `[env]`\n\
         # does not override an already-set variable unless `force = true`, which\n\
         # this deliberately does not use. A number a human states beats a number\n\
         # derived on their behalf.\n\n",
    );
    s.push_str(&format!(
        "# `{NOT_DERIVED_NEEDS_INFRA_COUNT}` is not derived here — the inventory cannot\n"
    ));
    s.push_str("# see whether this image enables the parameter or lifecycle service\n");
    s.push_str("# families, and a count short of them fails at registration.\n\n");
    s.push_str(
        "# The two `ZPICO_*` rows are FLOORED AT ONE: they size fixed C arrays in\n\
         # `zpico.c`, where zero is not a smaller pool (issue 1015). The floor is\n\
         # applied here, at the consumer, and not in the derivation -- the same\n\
         # derived counts reach the XRCE pools, where zero IS the answer and is\n\
         # worth 33,296 bytes of heap a slot (issue 1033).\n\n",
    );
    s.push_str("[env]\n");
    let floor = crate::entity_inventory::c_array_pool_floor;
    let vals: BTreeMap<&str, usize> = BTreeMap::from([
        ("NROS_EXECUTOR_ACTION_CLIENTS", knobs.heavy_slots),
        ("NROS_EXECUTOR_MAX_CBS", knobs.max_cbs),
        ("NROS_RMW_SUBSCRIBER_SLOTS", knobs.max_subscribers),
        ("ZPICO_MAX_PUBLISHERS", floor(knobs.max_publishers)),
        ("ZPICO_MAX_SUBSCRIBERS", floor(knobs.max_subscribers)),
    ]);
    for (k, v) in &vals {
        s.push_str(&format!("{k} = \"{v}\"\n"));
    }

    // Issue 1125 — the payload classes, from the second inventory. Appended to
    // the same `[env]` table rather than given their own: cargo merges nothing
    // across tables, and a leaf reads one environment.
    match payload {
        crate::leaf_payload_classes::PayloadClasses::Derived(p) => {
            s.push('\n');
            s.push_str(&format!(
                "# Payload classes, derived over the {} subscribing entit{} this leaf\n\
                 # declares (issue 1125). `large_count = 0` is an ANSWER -- it says every\n\
                 # type this leaf receives fits the small class, and `LARGE_PAYLOADS`\n\
                 # becomes zero bytes instead of RING_DEPTH x LARGE_SIZE. It is NOT\n\
                 # floored: `alloc_payload_block` bounds-checks the class index before it\n\
                 # subscripts the pool, so a zero-length one is never indexed\n\
                 # (phase-403 W4, and the reason issue 1015's floor does not apply here).\n",
                p.subscribed,
                if p.subscribed == 1 { "y" } else { "ies" }
            ));
            s.push_str(&format!(
                "ZPICO_MAX_LARGE_SUBSCRIBERS = \"{}\"\n",
                p.large_count
            ));
            // The small BLOCK moves with the count, and must: the shim routes
            // on `min(threshold, SUBSCRIBER_BUFFER_SIZE)` (issue 0841), so a
            // count derived against the 2048 split is only sound while the
            // block is sized to hold what that split called small.
            if p.small_max > 0 {
                s.push_str(&format!(
                    "NROS_SUBSCRIBER_BUFFER_SIZE = \"{}\"\n",
                    p.small_max
                ));
            }
            if p.large_count > 0 && p.large_max > 0 {
                s.push_str(&format!(
                    "ZPICO_SUBSCRIBER_LARGE_SIZE = \"{}\"\n",
                    p.large_max
                ));
            }
        }
        crate::leaf_payload_classes::PayloadClasses::Refused { reason } => {
            s.push_str("\n# Payload classes NOT derived (issue 1125); the crate defaults, which\n");
            s.push_str("# are LARGE rather than wrong, stand. A pool short of what this leaf\n");
            s.push_str("# receives is a SubscriberCreationFailed, not a smaller pool.\n");
            for line in reason.lines() {
                s.push_str(&format!("#   {line}\n"));
            }
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity_inventory::Derivation;

    const TALKER: &str = r#"{
      "version": 1, "package": "native_talker", "component": "talker",
      "nodes": [{ "id": "talker",
        "publishers": [{"id": "/chatter",
          "interface": {"package": "std_msgs", "name": "msg/String", "kind": "message"}}],
        "timers": [{"id": "on_tick"}],
        "subscribers": [], "services": [], "actions": [] }]
    }"#;

    #[test]
    fn a_publisher_and_a_timer_become_two_decls() {
        let (pkg, comp, d) = declaration_from_probe(TALKER).unwrap();
        assert_eq!((pkg.as_str(), comp.as_str()), ("native_talker", "talker"));
        let ents = d.entities();
        assert_eq!(ents.len(), 2, "{ents:?}");
        assert_eq!(ents[0].kind, EntityKind::Publisher);
        assert_eq!(ents[0].type_name.as_deref(), Some("std_msgs/msg/String"));
        assert_eq!(ents[0].name.as_deref(), Some("/chatter"));
        assert_eq!(ents[1].kind, EntityKind::Timer);
    }

    /// The probe running and finding nothing is a STATEMENT, not an absence.
    #[test]
    fn an_empty_probe_is_none_not_absent() {
        let (_, _, d) = declaration_from_probe(
            r#"{"package":"p","component":"c","nodes":[{"id":"n","publishers":[]}]}"#,
        )
        .unwrap();
        assert_eq!(d.tag(), "none");
        assert!(d.entities().is_empty());
    }

    /// Every key in the table reaches a kind. A kind handled here and forgotten
    /// there is how a pool silently goes short.
    #[test]
    fn every_probe_key_maps_to_a_kind() {
        let doc = r#"{"package":"p","component":"c","nodes":[{
          "publishers":[{"id":"a"}], "subscribers":[{"id":"b"}], "timers":[{"id":"c"}],
          "services":[{"id":"d"}], "service_clients":[{"id":"e"}],
          "actions":[{"id":"f"}], "action_clients":[{"id":"g"}] }]}"#;
        let (_, _, d) = declaration_from_probe(doc).unwrap();
        let kinds: Vec<_> = d.entities().iter().map(|e| e.kind).collect();
        for (_, want) in NODE_ENTITY_KEYS {
            assert!(
                kinds.contains(want),
                "{want:?} never produced by {NODE_ENTITY_KEYS:?}"
            );
        }
        assert_eq!(kinds.len(), NODE_ENTITY_KEYS.len());
    }

    /// An unreadable row REFUSES. Deriving a budget from a partial read is how
    /// a pool ends up shorter than the image, and short halts the board.
    #[test]
    fn an_unreadable_entity_row_refuses() {
        let doc = r#"{"package":"p","component":"c","nodes":[{"publishers":[42]}]}"#;
        let err = declaration_from_probe(doc).unwrap_err().to_string();
        assert!(err.contains("cannot read"), "{err}");
    }

    /// The derivation is the SHARED one, not a copy: a talker's publisher
    /// claims no callback slot and its timer does, which is `entity_inventory`'s
    /// rule and not this module's.
    #[test]
    fn counts_flow_through_the_shared_derivation() {
        let (pkg, comp, d) = declaration_from_probe(TALKER).unwrap();
        let mut inv = EntityInventory::new("t");
        inv.insert(ComponentEntities {
            pkg,
            component: comp.clone(),
            class: comp,
            declaration: d,
        });
        let Derivation::Derived(k) = inv.derive() else {
            panic!("expected a derivation from a stated declaration");
        };
        assert_eq!(k.entity_total, 2);
        assert_eq!(k.max_publishers, 1);
        // The DEMAND, unfloored: a talker declares no subscription, so it
        // demands none. Issue 1015 floored this in the shared derivation and
        // that reached the XRCE pools too, where a slot costs 33,296 bytes of
        // heap and zero is the measured answer (issue 1033). The floor now
        // lives at the `ZPICO_*` rows of the sidecar — see
        // `the_zpico_rows_are_floored_and_the_others_are_not`.
        assert_eq!(
            k.max_subscribers, 0,
            "the demand is the count, not a pool size"
        );
        assert_eq!(
            k.max_cbs, 1,
            "the timer claims the slot; the publisher does not"
        );
    }

    /// Issue 1015 + issue 1033 — the floor is the CONSUMER's, so it applies to
    /// the two knobs that size C arrays and to nothing else.
    ///
    /// A talker demands zero subscriptions. `ZPICO_MAX_SUBSCRIBERS` sizes
    /// `subscriber_entry_t subscribers[...]` in `zpico.c` and must still be 1;
    /// `NROS_RMW_SUBSCRIBER_SLOTS` sizes a Rust array, where zero is a legal
    /// pool that fails loudly at registration, and must carry the raw 0.
    #[test]
    fn the_zpico_rows_are_floored_and_the_others_are_not() {
        let (pkg, comp, d) = declaration_from_probe(TALKER).unwrap();
        let mut inv = EntityInventory::new("t");
        inv.insert(ComponentEntities {
            pkg,
            component: comp.clone(),
            class: comp,
            declaration: d,
        });
        let Derivation::Derived(k) = inv.derive() else {
            panic!("expected a derivation from a stated declaration");
        };
        let payload = crate::leaf_payload_classes::PayloadClasses::Derived(Default::default());
        let out = render_env_sidecar(&k, &payload, "test");
        // Whole rows, matched as text through `contains`, NOT through a local
        // `row(NAME)` helper: `config-knob-census` reads this file as a
        // build-time knob source and refuses an unknown callee taking a knob
        // name, which is how a `knob()` wrapper once took five knobs out of
        // that census. Measured — the helper version failed the gate.
        assert!(
            out.contains("ZPICO_MAX_SUBSCRIBERS = \"1\"\n"),
            "floored at one (issue 1015), not the raw 0:\n{out}"
        );
        assert!(
            out.contains("ZPICO_MAX_PUBLISHERS = \"1\"\n"),
            "the talker's one publisher:\n{out}"
        );
        assert!(
            out.contains("NROS_RMW_SUBSCRIBER_SLOTS = \"0\"\n"),
            "a Rust-backed pool takes the demand; only the C arrays are floored:\n{out}"
        );
    }

    // ---- issue 1061: the manifest declaration --------------------------

    fn write_leaf(dir: &std::path::Path, manifest: &str) {
        std::fs::write(dir.join("Cargo.toml"), manifest).unwrap();
    }

    #[test]
    fn a_manifest_declaration_parses_with_the_shared_grammar() {
        let td = tempfile::tempdir().unwrap();
        write_leaf(
            td.path(),
            r#"
[package]
name = "p"
[package.metadata.nros.component]
entities = ["publisher:std_msgs/msg/String:/chatter", "timer", "sub*2"]
"#,
        );
        let d = declared_entities(td.path()).unwrap().expect("declared");
        // `sub*2` is the repeat suffix -- the SHARED grammar's, not a second one.
        assert_eq!(d.len(), 4, "{d:?}");
        assert_eq!(d[0].kind, EntityKind::Publisher);
        assert_eq!(d[0].name.as_deref(), Some("/chatter"));
        assert_eq!(d[1].kind, EntityKind::Timer);
        assert_eq!(d[2].kind, EntityKind::Subscription);
        assert_eq!(d[3].kind, EntityKind::Subscription);
    }

    /// Absent and empty are DIFFERENT: absent means the leaf said nothing, empty
    /// means it asserted it creates nothing.
    #[test]
    fn an_absent_key_is_none_and_an_empty_list_is_some_empty() {
        let td = tempfile::tempdir().unwrap();
        write_leaf(td.path(), "[package]\nname = \"p\"\n");
        assert!(declared_entities(td.path()).unwrap().is_none());

        write_leaf(
            td.path(),
            "[package]\nname = \"p\"\n[package.metadata.nros.component]\nentities = []\n",
        );
        assert_eq!(declared_entities(td.path()).unwrap(), Some(vec![]));
    }

    #[test]
    fn a_malformed_declaration_names_the_entry() {
        let td = tempfile::tempdir().unwrap();
        write_leaf(
            td.path(),
            "[package]\nname = \"p\"\n[package.metadata.nros.component]\nentities = [\"nonsense\"]\n",
        );
        let e = declared_entities(td.path()).unwrap_err().to_string();
        assert!(e.contains("nonsense"), "{e}");

        write_leaf(
            td.path(),
            "[package]\nname = \"p\"\n[package.metadata.nros.component]\nentities = \"timer\"\n",
        );
        let e = declared_entities(td.path()).unwrap_err().to_string();
        assert!(e.contains("ARRAY"), "{e}");
    }

    /// The safety property: a declaration that disagrees with the code REFUSES.
    /// Without this the manifest becomes a way to state a budget smaller than
    /// the image, and short halts the board.
    #[test]
    fn a_declaration_that_disagrees_with_the_probe_refuses() {
        let probed = EntityDecl::parse("publisher").unwrap();
        let same = EntityDecl::parse("publisher").unwrap();
        assert!(reconcile("c", &same, &probed).is_ok());

        let fewer = EntityDecl::parse("timer").unwrap();
        let e = reconcile("c", &fewer, &probed).unwrap_err().to_string();
        assert!(e.contains("declares") && e.contains("creates"), "{e}");
        assert!(e.contains("timerx1") && e.contains("publisherx1"), "{e}");
    }

    /// Compared by KIND COUNT, not by row: a declaration may legitimately state
    /// a topic loosely, and the budget is computed from counts.
    #[test]
    fn reconcile_compares_counts_not_topic_names() {
        let probed = EntityDecl::parse("sub:std_msgs/msg/String:/chatter").unwrap();
        let declared = EntityDecl::parse("sub").unwrap();
        assert!(reconcile("c", &declared, &probed).is_ok());
    }

    #[test]
    fn the_sidecar_states_every_declared_key() {
        let (pkg, comp, d) = declaration_from_probe(TALKER).unwrap();
        let mut inv = EntityInventory::new("t");
        inv.insert(ComponentEntities {
            pkg,
            component: comp.clone(),
            class: comp,
            declaration: d,
        });
        let Derivation::Derived(k) = inv.derive() else {
            panic!()
        };
        let payload = crate::leaf_payload_classes::PayloadClasses::Derived(Default::default());
        let out = render_env_sidecar(&k, &payload, "metadata/talker.json");
        assert!(out.contains("[env]"));
        for key in DERIVED_ENV_KEYS {
            assert!(out.contains(key), "sidecar omits {key}:\n{out}");
        }
        assert!(out.contains("ZPICO_MAX_SUBSCRIBERS = \"1\""), "{out}");
        assert!(out.contains("ZPICO_MAX_PUBLISHERS = \"1\""), "{out}");
        // The infra-incomplete knob must NOT be stated as a value.
        assert!(
            !out.lines().any(|l| l.starts_with("ZPICO_MAX_QUERYABLES")),
            "the sidecar states a queryable count it cannot complete:\n{out}"
        );
        // No `force = true` KEY: a value the caller states must win. Checked
        // line-wise, because the header prose explains `force` and a substring
        // test would match the explanation rather than a setting.
        assert!(
            !out.lines().any(|l| l.trim_start().starts_with("force")),
            "sidecar sets `force`, so a caller's own value would be overridden:\n{out}"
        );
    }

    /// Issue 1125 — the publisher-only leaf that opened the issue. The count is
    /// stated as ZERO, which is the whole 131,072 B saving; the large SIZE is
    /// not stated at all, because a class with no blocks has no size worth
    /// naming.
    #[test]
    fn a_leaf_with_no_subscription_states_a_zero_large_count_and_no_large_size() {
        let (pkg, comp, d) = declaration_from_probe(TALKER).unwrap();
        let mut inv = EntityInventory::new("t");
        inv.insert(ComponentEntities {
            pkg,
            component: comp.clone(),
            class: comp,
            declaration: d,
        });
        let Derivation::Derived(k) = inv.derive() else {
            panic!()
        };
        let payload = crate::leaf_payload_classes::PayloadClasses::Derived(Default::default());
        let out = render_env_sidecar(&k, &payload, "metadata/talker.json");
        assert!(
            out.contains("ZPICO_MAX_LARGE_SUBSCRIBERS = \"0\""),
            "the derived zero must be STATED, not left to the crate default:\n{out}"
        );
        for k in ["ZPICO_SUBSCRIBER_LARGE_SIZE", "NROS_SUBSCRIBER_BUFFER_SIZE"] {
            assert!(
                !out.lines().any(|l| l.starts_with(k)),
                "sidecar names {k} for a class it just declared empty:\n{out}"
            );
        }
    }

    /// A leaf that DOES subscribe large states all three, and the small block
    /// is stated too — the count is only sound while the block holds what the
    /// split called small (issue 0841).
    #[test]
    fn a_large_subscription_states_the_count_the_size_and_the_small_block() {
        let (pkg, comp, d) = declaration_from_probe(TALKER).unwrap();
        let mut inv = EntityInventory::new("t");
        inv.insert(ComponentEntities {
            pkg,
            component: comp.clone(),
            class: comp,
            declaration: d,
        });
        let Derivation::Derived(k) = inv.derive() else {
            panic!()
        };
        let payload = crate::leaf_payload_classes::PayloadClasses::Derived(
            crate::leaf_payload_classes::DerivedPayloadClasses {
                large_count: 2,
                large_max: 40_000,
                small_max: 1500,
                subscribed: 3,
            },
        );
        let out = render_env_sidecar(&k, &payload, "metadata/talker.json");
        assert!(out.contains("ZPICO_MAX_LARGE_SUBSCRIBERS = \"2\""), "{out}");
        assert!(
            out.contains("ZPICO_SUBSCRIBER_LARGE_SIZE = \"40000\""),
            "{out}"
        );
        assert!(
            out.contains("NROS_SUBSCRIBER_BUFFER_SIZE = \"1500\""),
            "{out}"
        );
    }

    /// A refusal states NO payload key at all — it explains itself in a comment
    /// and leaves the crate defaults, which are large rather than wrong.
    #[test]
    fn a_refused_payload_join_states_no_payload_key() {
        let (pkg, comp, d) = declaration_from_probe(TALKER).unwrap();
        let mut inv = EntityInventory::new("t");
        inv.insert(ComponentEntities {
            pkg,
            component: comp.clone(),
            class: comp,
            declaration: d,
        });
        let Derivation::Derived(k) = inv.derive() else {
            panic!()
        };
        let payload = crate::leaf_payload_classes::PayloadClasses::Refused {
            reason: "std_msgs/msg/String (unbounded)".into(),
        };
        let out = render_env_sidecar(&k, &payload, "metadata/talker.json");
        for key in DERIVED_PAYLOAD_ENV_KEYS {
            assert!(
                !out.lines().any(|l| l.starts_with(key)),
                "a refused join states {key}:\n{out}"
            );
        }
        assert!(out.contains("# Payload classes NOT derived"), "{out}");
        // The entity half still lands — the two inventories refuse
        // independently, which is why they are two lists.
        assert!(out.contains("ZPICO_MAX_SUBSCRIBERS = \"1\""), "{out}");
    }
}
