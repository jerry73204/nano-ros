//! RFC-0052 / phase-296 W1 — SystemModel ingestion.
//!
//! `nros codegen-system --model system_model.yaml` consumes the checked
//! artifact play_launch `resolve` emits (RFC-0050): the model's execution
//! layer REPLACES the bringup's `[tiers.*]` + `[[node_overrides]]` before
//! the existing `resolve_system_tiers` pipeline runs — same resolver, same
//! `nros-plan.json`, same `run_tiers` output as the `system.toml`-authored
//! equivalent, by construction.
//!
//! Schema note: the tier tables exist twice by design — the shared
//! `ros-launch-manifest-sched::TierDef` (authoring + model schema, both
//! projects vendor it) and `nros-orchestration-ir::TierDef` (the no_std
//! resolver input the proc-macro also uses). This module is the ONE
//! conversion point; `tier_roundtrip_covers_every_field` is the
//! mirror-drift guard (issue-0160 lesson, applied cross-repo).

use std::{collections::BTreeMap, path::Path};

use eyre::{Result, WrapErr, bail};
// RFC-0052 — `tier_from_model` moved to nros-orchestration-ir (the shared home
// for the CLI codegen + the nros::main! proc-macro); re-exported here so the
// existing `model_ingest::tier_from_model` call sites and tests keep working.
pub use nros_orchestration_ir::tier_from_model;
use nros_orchestration_ir::{CallbackGroupDecl, CallbackGroupOverride, NodeOverride};
use ros_launch_manifest_model::SystemModel;

use serde_json::Value as JsonValue;

use crate::orchestration::{cargo_metadata_schema::SystemToml, source_metadata::SourceMetadata};

/// Load + schema-gate a SystemModel, RESOLVING it from the bringup's inputs
/// when no build has produced one.
///
/// The model is an intermediate artifact (phase-330): a consumer asks for it by
/// its inputs and never requires the user to have produced one. This is the
/// same rule `nros::main!` follows — `model_location::ensure_model` is the one
/// implementation both use — extended here so the CMake path gets it too.
/// `nano_ros_entry(LAUNCH …)` shells out to `nros codegen entry`, which landed
/// on this function and died with
///
/// ```text
/// Error: read SystemModel …/config/system_model.yaml
///   No such file or directory (os error 2)
/// ```
///
/// for a bringup whose `system.toml` and launch file were sitting right there
/// (issue 0414, the CMake half).
///
/// A conventional model path is `<bringup>/config/<name>.yaml`, which is how
/// `model_location` builds its search paths in the first place — so the bringup
/// is recoverable from the path when the file is absent. A path of any other
/// shape (a temp file a caller resolved itself) exists by construction, so the
/// fallback never sees it.
pub fn load_model(path: &Path) -> Result<SystemModel> {
    let resolved: std::borrow::Cow<'_, Path> = if path.is_file() {
        std::borrow::Cow::Borrowed(path)
    } else {
        let bringup_dir = path
            .parent()
            .and_then(|config| config.parent())
            .ok_or_else(|| eyre::eyre!("read SystemModel {}: no such file", path.display()))?;
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .ok_or_else(|| eyre::eyre!("read SystemModel {}: no file name", path.display()))?;
        let model_rel = format!("config/{name}");
        let (produced, _inputs) =
            nros_orchestration_ir::model_location::ensure_model(bringup_dir, &model_rel)
                .map_err(|e| eyre::eyre!("resolve SystemModel for {}: {e}", path.display()))?;
        std::borrow::Cow::Owned(produced)
    };
    let yaml = std::fs::read_to_string(resolved.as_ref())
        .wrap_err_with(|| format!("read SystemModel {}", resolved.display()))?;
    SystemModel::from_yaml_str(&yaml)
        .map_err(|e| eyre::eyre!("load SystemModel {}: {e}", resolved.display()))
}

/// Bare node name from a model FQN (`/ns/node` → `node`).
fn bare(fqn: &str) -> &str {
    fqn.rsplit('/').next().unwrap_or(fqn)
}

/// Apply the model's execution layer onto the bringup system: tiers
/// replace `[tiers.*]`, bindings become `[[node_overrides]]` rows.
///
/// Binding keys are `<node FQN>` (whole node → every declared group) or
/// `<node FQN>/<callback group>`. Fail-loud (RFC-0052): a binding that
/// names no known component, or a group the node never declares, is an
/// error — never a silent no-op.
pub fn apply_model_execution(
    system: &mut SystemToml,
    model: &SystemModel,
    target_rtos: &str,
    callback_groups: &BTreeMap<String, Vec<CallbackGroupDecl>>,
) -> Result<()> {
    system.tiers = model
        .execution
        .tiers
        .iter()
        .map(|(name, t)| (name.clone(), tier_from_model(t, target_rtos)))
        .collect();

    let component_names: Vec<&str> = system.components.iter().map(|c| c.name.as_str()).collect();

    let mut overrides: BTreeMap<String, Vec<CallbackGroupOverride>> = BTreeMap::new();
    for (key, tier) in &model.execution.bindings {
        if !model.execution.tiers.contains_key(tier) {
            bail!("SystemModel binding '{key}' references undeclared tier '{tier}'");
        }
        // `<FQN>/<group>` vs `<FQN>`: the segment before the last is the
        // node when the last segment matches one of its declared groups.
        let (node, groups): (String, Vec<String>) = {
            let node_level = bare(key);
            if component_names.contains(&node_level) {
                // whole-node binding → every declared group (or the
                // implicit default group when none are declared).
                let declared = callback_groups
                    .get(node_level)
                    .map(|gs| gs.iter().map(|g| g.id.clone()).collect::<Vec<_>>())
                    .unwrap_or_default();
                if declared.is_empty() {
                    bail!(
                        "SystemModel binding '{key}': node '{node_level}' declares no \
                         callback groups to bind (whole-node tiering needs at least \
                         the node's group declarations in its package metadata)"
                    );
                }
                (node_level.to_string(), declared)
            } else if let Some((node_part, group)) = key.rsplit_once('/')
                && !bare(node_part).is_empty()
            {
                let node = bare(node_part).to_string();
                if !component_names.contains(&node.as_str()) {
                    bail!(
                        "SystemModel binding '{key}': no component named '{node}' \
                         in the bringup (components: {component_names:?})"
                    );
                }
                let declared = callback_groups.get(&node);
                if !declared.is_some_and(|gs| gs.iter().any(|g| g.id == group)) {
                    bail!(
                        "SystemModel binding '{key}': node '{node}' declares no \
                         callback group '{group}'"
                    );
                }
                (node, vec![group.to_string()])
            } else {
                bail!(
                    "SystemModel binding '{key}': no component named '{node_level}' \
                     in the bringup (components: {component_names:?})"
                );
            }
        };
        let entry = overrides.entry(node).or_default();
        for g in groups {
            entry.push(CallbackGroupOverride {
                id: g,
                tier: tier.clone(),
            });
        }
    }
    // ISSUE 0398 — the other direction, which was silent.
    //
    // A binding that names no component already bails above. The reverse never
    // did: a `[[component]]` that declares `group_tiers` and matches NO launch
    // node produces no binding at all, so there is nothing here to reject, and
    // the node quietly runs on the default tier.
    //
    // That is not hypothetical. `[[component]].name` is an instance id that must
    // be unique across a bringup, while the launch node keeps the plain role
    // name — phase-331's consolidation made all 20 of `features/`'s component
    // names differ from all 8 of its launch node names. The params projection
    // survived only because rlm v0.1.2 added a package-based fallback and emits
    // a diagnostic when even that misses; `group_tiers` has neither. The first
    // consolidated workspace to declare one would have lost its tiers with no
    // trace.
    //
    // The matching rule itself belongs upstream (the resolver decides which node
    // a component binds to). What belongs HERE is refusing to bake a declaration
    // that reached nothing.
    // ABSENT-IN-VARIANT is not the same as RENAMED, and only the second is a
    // defect. A bringup is a CATALOG: `features/` declares 20 components and
    // each of its 8 launches uses a handful, so most components legitimately
    // reach no node in any given model (`realtime-cpp`'s `aux_node` is in the
    // freertos launch and not the native one). Failing on that would make a
    // multi-variant bringup unbakeable.
    //
    // The rename hazard has a signature that tells them apart: the model DOES
    // contain a node from the component's package, under a different name. That
    // is a component whose node is right there and did not match — the phase-331
    // shape — and it is an error. A package with no node in this model is simply
    // not part of this variant.
    let bound: std::collections::BTreeSet<&str> = overrides.keys().map(|s| s.as_str()).collect();
    let renamed: Vec<String> = system
        .components
        .iter()
        .filter(|c| !c.group_tiers.is_empty() && !bound.contains(c.name.as_str()))
        .filter_map(|c| {
            let node = model
                .structure
                .nodes
                .iter()
                .find(|(_, n)| n.pkg.as_deref() == Some(c.pkg.as_str()))?;
            Some(format!(
                "`{}` (its package `{}` runs in this model as `{}`)",
                c.name, c.pkg, node.0
            ))
        })
        .collect();
    if !renamed.is_empty() {
        bail!(
            "these `[[component]]`s declare `group_tiers` that reached no node, while a node \
             of the SAME PACKAGE is in the resolved model: {}. A component binds to a launch \
             node by NAME (`[[component]].name` vs the node's `name=`), so a component renamed \
             for workspace-wide uniqueness stops matching its own node and its tiers fall back \
             to the default silently — issue 0398. Give the node the component's name, or bind \
             it with an explicit `[[node_overrides]]`. (A component whose package has no node \
             here is simply absent from this variant, and is not an error.)",
            renamed.join(", ")
        );
    }

    system.node_overrides = overrides
        .into_iter()
        .map(|(name, callback_groups)| NodeOverride {
            name,
            callback_groups,
        })
        .collect();
    Ok(())
}

/// phase-296 W5.5 follow-up — the RFC-0052 realizer as the DERIVED-schedule
/// path: when the model declares NO `execution.tiers`, derive per-node tiers
/// from the contract layer (`node_paths` + criticality) via the shared
/// platform-agnostic core + the RTOS realizer, and synthesize them into the
/// bringup as ordinary `[tiers.*]` + `[[node_overrides]]` rows so the ENTIRE
/// existing pipeline (resolve_system_tiers → validation → plan → run_tiers)
/// consumes them unchanged. Declared tiers always win — this only engages on
/// an empty tier table.
///
/// The board capability (`SchedCaps`) honors the per-deploy `edf` knob
/// (`Deploy.extra["edf"]`, RFC-0052 §"CAPS provenance"): entries carrying the
/// knob must agree, else the bake fails loud. Every degradation the realizer
/// records is printed — a guarantee weakening is never silent.
///
/// Returns the number of derived tiers (0 = nothing schedulable; the bake
/// proceeds tier-less exactly as before).
pub fn derive_execution_from_contracts(
    system: &mut SystemToml,
    model: &SystemModel,
    target_rtos: &str,
    callback_groups: &BTreeMap<String, Vec<CallbackGroupDecl>>,
) -> Result<(usize, Vec<crate::orchestration::plan::PlanSchedWarning>)> {
    // The derive CORE lives in `nros-orchestration-ir` (shared with the
    // `nros::main!` proc-macro so pure-cargo Rust entries derive identically).
    // This wrapper surfaces the recorded degradations + groupless notes on
    // stderr and mutates the bringup `SystemToml` — behavior-identical to the
    // pre-relocation inline version.
    let derived = nros_orchestration_ir::derive::derive_tiers_from_contracts(
        model,
        target_rtos,
        callback_groups,
    );
    // Issue 0259 — surface on stderr AND carry into the plan. stderr is for the
    // person watching this bake; the plan is for everyone who reads the system
    // afterwards, and a verdict that exists only in scrollback cannot be
    // audited.
    let mut warnings = Vec::new();
    for d in &derived.degradations {
        eprintln!(
            "codegen-system: derived-schedule degradation — {} [{}]: {}",
            d.node, d.dim, d.reason
        );
        warnings.push(crate::orchestration::plan::PlanSchedWarning {
            node: d.node.clone(),
            dim: d.dim.to_string(),
            reason: d.reason.clone(),
        });
    }
    for name in &derived.groupless_notes {
        eprintln!(
            "codegen-system: derived-schedule note — node '{}' declares no \
             callback groups; it stays on the default tier",
            name
        );
    }
    let n = derived.tiers.len();
    system.tiers = derived.tiers;
    system.node_overrides = derived.overrides;
    Ok((n, warnings))
}

/// Issue 0257 — read `[package.metadata.nros.entry] max_callbacks` from an
/// entry pkg's `Cargo.toml`. `None` for a non-Cargo (C/C++) bringup, an absent
/// key, or anything unparseable: the caller then compares against the build
/// default, which is exactly what such an entry compiles with.
pub fn declared_max_callbacks(manifest_path: &Path) -> Option<usize> {
    if manifest_path.file_name()? != "Cargo.toml" {
        return None;
    }
    let raw = std::fs::read_to_string(manifest_path).ok()?;
    let v: toml::Value = toml::from_str(&raw).ok()?;
    let n = v
        .get("package")?
        .get("metadata")?
        .get("nros")?
        .get("entry")?
        .get("max_callbacks")?
        .as_integer()?;
    (n > 0).then_some(n as usize)
}

/// phase-307 W4 — `(package, executable)` → recorded callback slots, the key
/// `SystemModel::structure.nodes` entries carry as `pkg` + `exec`.
///
/// The accounting itself lives in `nros_orchestration_ir::sidecar_slots`, NOT
/// here: the `nros::main!` macro reads the same sidecars, and the two bakes
/// must agree — this one REFUSES an over-capacity system while the macro SIZES
/// the executor, so a disagreement is an image that passes this check and dies
/// at boot anyway. Same reasoning that already shares `count_callbacks`.
pub fn metadata_slot_counts(metadata: &[JsonValue]) -> BTreeMap<(String, String), usize> {
    let mut out = BTreeMap::new();
    for md in metadata {
        let Some((key, slots)) = nros_orchestration_ir::sidecar_slots::slots_of_component(md)
        else {
            continue;
        };
        // A component declaring several nodes contributes all of them under
        // one executable; several sidecars for one executable would be a
        // producer bug, so take the max rather than silently halving a count.
        let entry = out.entry(key).or_insert(0usize);
        *entry = (*entry).max(slots);
    }
    out
}

/// Read + parse every source-metadata sidecar the workspace carries.
///
/// Unparseable sidecars are SKIPPED with a warning rather than failing the
/// bake: the fallback is the SystemModel bound, which is merely less precise,
/// and a bake that dies because a stale sidecar exists would be a worse
/// failure than the one this phase set out to fix.
pub fn load_workspace_metadata(ws_root: &Path) -> Vec<JsonValue> {
    let Ok(workspace) = crate::orchestration::workspace::Workspace::discover(ws_root) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for path in workspace.source_metadata_files() {
        // Parsed TYPED first — that parse is the schema gate, and it is why a
        // garbage or schema-drifted sidecar is skipped instead of silently
        // counted as zero. The counting then runs on the raw value through the
        // shared rule, so the macro (which cannot dep this crate for the typed
        // schema) and this bake do the same arithmetic.
        match std::fs::read_to_string(&path)
            .map_err(|e| e.to_string())
            .and_then(|raw| {
                serde_json::from_str::<SourceMetadata>(&raw)
                    .map_err(|e| e.to_string())
                    .and_then(|_| {
                        serde_json::from_str::<JsonValue>(&raw).map_err(|e| e.to_string())
                    })
            }) {
            Ok(md) => out.push(md),
            Err(err) => eprintln!(
                "codegen-system: ignoring unreadable source metadata {}: {err}",
                path.display()
            ),
        }
    }
    out
}

/// phase-307 W4 — the entity count the capacity check runs on:
/// `max(model_wiring, recorded_metadata)` PER NODE, summed.
///
/// Neither source is complete on its own, which is why the rule is a max and
/// not a choice:
///
/// - the model has no timer entity, so a `talker` that publishes on a 500 ms
///   timer counts ZERO in the model and ONE in the recorder — the exact hole
///   issue 0257 documented;
/// - the recorder only sees what `Component::register` declares as an entity,
///   while the model's wiring names things the recorder never runs to see.
///
/// That second bullet used to read "service/action CLIENTS are not recorded as
/// node entities". That is no longer true (issue 0900): the recorder always
/// captured them and the SIDECAR WRITER dropped them on the way out, which is
/// fixed — `action_clients` and `service_clients` are emitted arrays now. The
/// max rule still holds, for the reason above and for the timer hole below; it
/// no longer holds for the client reason, and a stale rationale is how the next
/// reader concludes the sidecar cannot answer a question it now can.
///
/// Taking the max per node is monotone (never below today's model bound, so no
/// existing build regresses) and never over-counts a node by mixing the two
/// sources' blind spots together.
///
/// The rule itself lives in `nros-orchestration-ir` because the `nros::main!`
/// macro applies it too (phase-307 W4 second half). This is only the CLI's
/// adapter from a materialised map; a disagreement between the two would be an
/// image that passes the bake's check and dies at boot anyway.
pub fn count_callbacks_with_metadata(
    model: &SystemModel,
    slots: &BTreeMap<(String, String), usize>,
) -> usize {
    nros_orchestration_ir::executor_sizing::count_callbacks_with_recorded(
        model,
        |_| true,
        |pkg, exec| {
            slots
                .get(&(pkg.to_string(), exec.to_string()))
                .copied()
                .unwrap_or(0)
        },
    )
}

/// Issue 0257 — the CLI twin of the `nros::main!` capacity check: refuse to
/// bake a system whose modelled entity count cannot fit the executor callback
/// table the image will compile with.
///
/// Capacity mirrors the macro exactly (shared derivation in
/// `nros_orchestration_ir::executor_sizing`, so the two bakes cannot drift):
///
/// - an entry that declares `max_callbacks` is sized by it;
/// - otherwise a board that honors per-entry sizing gets the DERIVED size, so
///   it always fits and the check is vacuous;
/// - every other board opens at the build-time `NROS_EXECUTOR_MAX_CBS`
///   (read from the env when the bake sets it, else the default).
///
/// A model with no wiring counts zero and is never checked — the pre-0257
/// bake, unchanged.
/// phase-400 W6 — the ladder's answer for `executor.max_cbs`, or `None` when no
/// platform tree is reachable.
///
/// Deliberately tolerant: this is a CHECK, and a check that hard-fails because
/// it could not find a config tree is a worse outcome than one that falls back
/// to the documented default.
fn resolve_max_cbs_through_ladder(platform: Option<&str>) -> Option<usize> {
    use nros_board_common::platform_config::PlatformsTree;
    use nros_orchestration_ir::executor_sizing as sz;

    let platform = platform?;
    let mut dir = std::env::current_dir().ok()?;
    let repo = loop {
        if dir.join("nros-sdk-index.toml").exists() {
            break dir;
        }
        if !dir.pop() {
            return None;
        }
    };
    let path = PlatformsTree::default_search_path(
        &repo,
        std::env::var("NROS_PLATFORMS_DIR").ok().as_deref(),
    );
    let tree = PlatformsTree::load_search_path(&path).ok()?;
    let env_get = |name: &str| std::env::var(name).ok().filter(|v| !v.is_empty());
    let resolved = tree
        .resolve_executor(
            platform,
            None,
            &env_get,
            &[("max_cbs", sz::DEFAULT_MAX_CBS)],
        )
        .ok()?;
    resolved
        .into_iter()
        .find(|(n, _)| *n == "max_cbs")
        .map(|(_, r)| r.value)
}

pub fn check_executor_capacity(
    model: &SystemModel,
    deploy_key: Option<&str>,
    declared: Option<usize>,
    slots: &BTreeMap<(String, String), usize>,
) -> Result<()> {
    use nros_orchestration_ir::executor_sizing as sz;

    // phase-307 W4 — exact where a sidecar exists, model bound where it does
    // not. With no sidecars this is byte-identical to the pre-307 check.
    let counted = count_callbacks_with_metadata(model, slots);
    if counted == 0 {
        return Ok(());
    }
    // phase-400 W6/W8 — resolve through the LADDER, not the env alone.
    //
    // This read used to be `std::env::var("NROS_EXECUTOR_MAX_CBS")`, which was
    // correct while env was the knob's only home. Now that the knob has
    // platform and board rungs, reading the env directly makes this check
    // disagree with the value the build actually bakes: a platform-set
    // `max_cbs` would be invisible here and the model would be validated
    // against the wrong capacity. That is the second-reader hazard
    // `check-knob-single-reader` exists to catch, and it caught this one.
    //
    // Falls back to the previous behaviour when no platform tree is reachable
    // (an out-of-tree consumer, or no platform named), because a capacity check
    // that refuses to run is worse than one using the built-in default.
    let build_default = resolve_max_cbs_through_ladder(deploy_key).unwrap_or_else(|| {
        nros_board_common::platform_config::executor_env_only("max_cbs", sz::DEFAULT_MAX_CBS)
    });
    let (capacity, fix) = match declared {
        Some(n) => (
            n,
            format!(
                "raise `[package.metadata.nros.entry] max_callbacks` (currently {n}) to at \
                 least {}",
                sz::derive_max_callbacks(counted)
            ),
        ),
        None if deploy_key.is_some_and(sz::board_honors_entry_sizing) => (
            sz::derive_max_callbacks(counted).max(build_default),
            String::new(),
        ),
        None => (
            build_default,
            format!(
                "set `NROS_EXECUTOR_MAX_CBS` (currently {build_default}) to at least {} in \
                 the build env, and rebuild from clean — resizing the executor arena mixes \
                 stale objects",
                sz::derive_max_callbacks(counted)
            ),
        ),
    };
    if counted > capacity {
        bail!(
            "codegen-system: the SystemModel registers {counted} callback entities but the \
             executor callback table holds {capacity}: {fix}. issue 0257"
        );
    }
    Ok(())
}

/// Issue 0284 — resolve the CycloneDDS type-registry size for a bake, mirroring
/// [`check_executor_capacity`]'s "size it or fail loud" shape. Unlike the
/// callback count the model is COMPLETE for types (only pub/sub/service/action
/// register DDS types — timers/guard conditions register none), so no sidecar
/// union is needed.
///
/// Returns the value the bake should EMIT as `NROS_CYCLONEDDS_MAX_TYPES` (a power
/// of two `>=` the distinct-type count), or `None` when nothing needs emitting:
///
/// - **env unset** (the common case): auto-size. `Some(derived)` when the derived
///   size exceeds the build-time default, else `None` (a small system keeps the
///   default 32 → byte-identical, no config churn).
/// - **env set by the user**: respect it. `None`, UNLESS the user's value is
///   smaller than the count — then `bail!` loudly (they explicitly under-sized;
///   auto-emit would silently override their intent).
///
/// A model with no DDS-registering wiring counts zero and returns `None`. On a
/// non-CycloneDDS image the env is inert (the registry crate isn't linked), so
/// an emitted value is harmless; the gate half only ever fires on a genuinely
/// over-`N` type set the user pinned too small.
pub fn resolve_cyclonedds_max_types(model: &SystemModel) -> Result<Option<usize>> {
    let user_pin = std::env::var("NROS_CYCLONEDDS_MAX_TYPES")
        .ok()
        .map(|raw| raw.parse::<usize>().unwrap_or(0));
    resolve_cyclonedds_max_types_with(model, user_pin)
}

/// Pure core of [`resolve_cyclonedds_max_types`] — `user_pin` is the parsed
/// `NROS_CYCLONEDDS_MAX_TYPES` value (`None` = unset). Split out so the env read
/// stays at the edge and the policy is deterministically testable.
pub fn resolve_cyclonedds_max_types_with(
    model: &SystemModel,
    user_pin: Option<usize>,
) -> Result<Option<usize>> {
    use nros_orchestration_ir::cyclonedds_type_sizing as ty;

    let counted = ty::count_dds_types(model, |_| true);
    if counted == 0 {
        return Ok(None);
    }
    match user_pin {
        // User pinned it — respect, but refuse a known-too-small value.
        Some(user) => {
            if counted > user {
                bail!(
                    "codegen-system: the SystemModel registers {counted} distinct DDS types but \
                     `NROS_CYCLONEDDS_MAX_TYPES` is pinned to {user}: raise it to at least {} (a \
                     power of two) in the build env and rebuild, or unset it to let nros size it. \
                     issue 0284",
                    ty::derive_max_types(counted)
                );
            }
            Ok(None)
        }
        // Unset — auto-size when the model needs more than the default.
        None => {
            let derived = ty::derive_max_types(counted);
            Ok((derived > ty::DEFAULT_MAX_TYPES).then_some(derived))
        }
    }
}

/// Issue 0284 — insert / update / remove the nros-managed
/// `NROS_CYCLONEDDS_MAX_TYPES` in `<workspace>/.cargo/config.toml`'s `[env]`
/// table (cargo reads it for the whole build, so the dep crate's `option_env!`
/// picks it up), format-preserving (`toml_edit`). `value`:
///
/// - `Some(n)` — set `NROS_CYCLONEDDS_MAX_TYPES = { value = "n", force = true }`
///   (`force` overrides a stale ambient env). Tagged `# nros-managed` so re-bake
///   evicts only its own.
/// - `None` — remove a previously-managed line (model shrank back under the
///   default); leave a user's UN-managed hand-set line untouched.
///
/// Returns whether the file changed. A user's own `[env]` entry (no
/// `nros-managed` tag) is never clobbered — the emit is skipped, leaving their
/// value authoritative.
pub fn manage_cyclonedds_max_types(workspace: &Path, value: Option<usize>) -> Result<bool> {
    use toml_edit::{DocumentMut, Item, Value};
    const KEY: &str = "NROS_CYCLONEDDS_MAX_TYPES";
    const TAG: &str = "nros-managed";

    let cfg_dir = workspace.join(".cargo");
    let cfg = cfg_dir.join("config.toml");
    let text = std::fs::read_to_string(&cfg).unwrap_or_default();
    let mut doc: DocumentMut = text
        .parse()
        .wrap_err_with(|| format!("codegen-system: parse {}", cfg.display()))?;

    // Is an existing entry ours (tagged) vs a user's hand-set line?
    let existing = doc
        .get("env")
        .and_then(|e| e.as_table())
        .and_then(|t| t.get(KEY));
    let existing_is_managed = existing
        .and_then(|it| it.as_value())
        .and_then(|v| v.decor().suffix())
        .and_then(|s| s.as_str())
        .map(|s| s.contains(TAG))
        .unwrap_or(false);

    match value {
        Some(_) if existing.is_some() && !existing_is_managed => {
            // A user pinned it in the config by hand — respect it, don't clobber.
            return Ok(false);
        }
        Some(n) => {
            let mut inline = toml_edit::InlineTable::new();
            inline.insert("value", Value::from(n.to_string()));
            inline.insert("force", Value::from(true));
            let mut v = Value::InlineTable(inline);
            v.decor_mut().set_suffix(format!(
                "  # {TAG} (issue 0284; derived from the SystemModel)"
            ));
            let env = doc
                .as_table_mut()
                .entry("env")
                .or_insert(Item::Table(toml_edit::Table::new()));
            let env = env.as_table_mut().ok_or_else(|| {
                eyre::eyre!("codegen-system: [env] is not a table in {}", cfg.display())
            })?;
            env[KEY] = Item::Value(v);
        }
        None => {
            if existing_is_managed
                && let Some(env) = doc.get_mut("env").and_then(|e| e.as_table_mut())
            {
                env.remove(KEY);
                if env.is_empty() {
                    doc.as_table_mut().remove("env");
                }
            }
        }
    }

    let out = doc.to_string();
    if out == text {
        return Ok(false);
    }
    std::fs::create_dir_all(&cfg_dir)
        .wrap_err_with(|| format!("codegen-system: mkdir {}", cfg_dir.display()))?;
    // issue 0562 — a leaf `.cargo/config.toml` restamp is the expensive one:
    // cargo re-fingerprints the whole leaf behind it.
    crate::atomic_file::atomic_write(&cfg, &out)
        .wrap_err_with(|| format!("codegen-system: write {}", cfg.display()))?;
    Ok(true)
}

#[cfg(test)]
mod executor_capacity_tests {
    //! Issue 0257 — the CLI twin of the `nros::main!` capacity check.
    use ros_launch_manifest_model::{ServiceWiring, SystemModel, TopicWiring};

    use std::collections::BTreeMap;

    use super::check_executor_capacity;

    /// The pre-307 world: no sidecars ⇒ the model bound alone.
    fn no_metadata() -> BTreeMap<(String, String), usize> {
        BTreeMap::new()
    }

    /// phase-307 W4 — a node instance the sidecar key `(pkg, exec)` matches.
    fn with_node(mut m: SystemModel, fqn: &str, pkg: &str, exec: &str) -> SystemModel {
        m.structure.nodes.insert(
            fqn.to_string(),
            ros_launch_manifest_model::NodeInstance {
                pkg: Some(pkg.to_string()),
                exec: Some(exec.to_string()),
                ..Default::default()
            },
        );
        m
    }

    fn slots(rows: &[(&str, &str, usize)]) -> BTreeMap<(String, String), usize> {
        rows.iter()
            .map(|(p, e, n)| ((p.to_string(), e.to_string()), *n))
            .collect()
    }

    /// The 0257 hole, closed: a node whose only modelled entity is one
    /// subscription but which ALSO registers timers the model has no entity
    /// for. The model bound says 1 and fits; the recorded count says 6 and
    /// does not — and the bake must fail on the truth, not the bound.
    #[test]
    fn recorded_timers_raise_the_count_past_the_model_bound() {
        let model = with_node(model_with(1), "/listener0", "listener_pkg", "listener");
        check_executor_capacity(&model, Some("zephyr"), None, &no_metadata())
            .expect("model bound alone: 2 entities fit the default 4");
        let err = check_executor_capacity(
            &model,
            Some("zephyr"),
            None,
            &slots(&[("listener_pkg", "listener", 6)]),
        )
        .expect_err("6 recorded slots on one node do not fit 4")
        .to_string();
        assert!(err.contains("registers 7 callback entities"), "got: {err}");
    }

    /// Monotone: metadata may only ever RAISE a node's count. A sidecar that
    /// records fewer entities than the wiring (service/action clients are
    /// modelled but not recorded) must not shrink the bake's count.
    #[test]
    fn metadata_never_lowers_the_model_bound() {
        let model = with_node(model_with(8), "/adder", "adder_pkg", "adder");
        let err = check_executor_capacity(
            &model,
            Some("zephyr"),
            None,
            &slots(&[("adder_pkg", "adder", 0)]),
        )
        .expect_err("still 9")
        .to_string();
        assert!(err.contains("registers 9 callback entities"), "got: {err}");
    }

    /// `n` subscribers + one service server = `n + 1` callback entities.
    fn model_with(n: usize) -> SystemModel {
        let mut m = SystemModel::default();
        m.structure.topics.insert(
            "/chatter".into(),
            TopicWiring {
                msg_type: "std_msgs/msg/String".into(),
                publishers: vec!["/talker/chatter".into()],
                subscribers: (0..n).map(|i| format!("/listener{i}/chatter")).collect(),
            },
        );
        m.structure.services.insert(
            "/add".into(),
            ServiceWiring {
                srv_type: "example_interfaces/srv/AddTwoInts".into(),
                server: vec!["/adder/add".into()],
                client: vec![],
            },
        );
        m
    }

    #[test]
    fn wiring_free_model_is_never_checked() {
        // The pre-0257 bake for every in-tree example.
        check_executor_capacity(
            &SystemModel::default(),
            Some("zephyr"),
            None,
            &no_metadata(),
        )
        .expect("nothing to count");
    }

    #[test]
    fn over_capacity_model_on_a_firmware_board_fails_the_bake() {
        let err = check_executor_capacity(&model_with(8), Some("zephyr"), None, &no_metadata())
            .expect_err("9 entities do not fit the default 4")
            .to_string();
        assert!(err.contains("registers 9 callback entities"), "got: {err}");
        assert!(err.contains("NROS_EXECUTOR_MAX_CBS"), "got: {err}");
        assert!(err.contains("at least 12"), "got: {err}");
        assert!(err.contains("0257"), "got: {err}");
    }

    #[test]
    fn declared_max_callbacks_below_the_count_fails_the_bake() {
        let err = check_executor_capacity(&model_with(8), Some("posix"), Some(4), &no_metadata())
            .expect_err("declared 4 does not fit 9")
            .to_string();
        assert!(err.contains("max_callbacks"), "got: {err}");
        assert!(err.contains("at least 12"), "got: {err}");
    }

    #[test]
    fn declared_max_callbacks_above_the_count_passes() {
        check_executor_capacity(&model_with(8), Some("zephyr"), Some(32), &no_metadata())
            .expect("32 fits 9");
    }

    #[test]
    fn sizing_honoring_board_derives_its_way_out() {
        // posix opens via `run_with_deploy_sized`, so the derived size applies
        // and no operator action is needed — mirroring the macro.
        check_executor_capacity(&model_with(8), Some("posix"), None, &no_metadata())
            .expect("derived sizing covers the count on a hosted board");
    }
}

#[cfg(test)]
mod plan_record_tests {
    use ros_launch_manifest_model::{NodeInstance, SystemModel};

    /// Precondition #1 of the launch_synth deletion — a model with a
    /// container + composable child + plain node classifies into the
    /// record's three arrays, and the child links to its container via
    /// `target_container_name` (the shape the planner resolves).
    #[test]
    fn model_record_classifies_container_composable_and_plain() {
        let mut m = SystemModel::default();
        m.structure.nodes.insert(
            "/perc/pipeline_container".into(),
            NodeInstance {
                pkg: Some("rclcpp_components".into()),
                exec: Some("component_container".into()),
                is_container: true,
                ..Default::default()
            },
        );
        m.structure.nodes.insert(
            "/perc/detector".into(),
            NodeInstance {
                pkg: Some("detector_pkg".into()),
                plugin: Some("detector_pkg::Detector".into()),
                container: Some("/perc/pipeline_container".into()),
                ..Default::default()
            },
        );
        m.structure.nodes.insert(
            "/talker".into(),
            NodeInstance {
                pkg: Some("talker_pkg".into()),
                exec: Some("talker".into()),
                ..Default::default()
            },
        );

        let rec = super::plan_record_from_model(&m);
        let arr = |k: &str| rec.get(k).unwrap().as_array().unwrap().clone();
        assert_eq!(arr("node").len(), 1, "one plain node: {rec}");
        assert_eq!(arr("container").len(), 1, "one container: {rec}");
        assert_eq!(arr("load_node").len(), 1, "one composable child: {rec}");
        let c = &arr("container")[0];
        assert_eq!(c["name"], "pipeline_container");
        assert_eq!(c["namespace"], "/perc");
        let l = &arr("load_node")[0];
        assert_eq!(l["plugin"], "detector_pkg::Detector");
        assert_eq!(l["target_container_name"], "/perc/pipeline_container");
        assert_eq!(arr("node")[0]["executable"], "talker");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binding_to_unknown_tier_or_node_fails_loud() {
        use ros_launch_manifest_model::SystemModel;
        let mut system: SystemToml =
            toml::from_str("[system]\nname = \"t\"\nrmw = \"zenoh\"\ndomain_id = 0\n")
                .expect("minimal system.toml parses");
        let mut model = SystemModel::default();
        model
            .execution
            .bindings
            .insert("/ctrl/ctrl_node".to_string(), "high".to_string());
        let err = apply_model_execution(&mut system, &model, "posix", &BTreeMap::new())
            .unwrap_err()
            .to_string();
        assert!(err.contains("undeclared tier 'high'"), "got: {err}");
    }

    fn contract_model() -> ros_launch_manifest_model::SystemModel {
        use ros_launch_manifest_model::{
            Contracts, NodeInstance, PathContract, Structure, SystemModel,
        };
        let mut nodes = indexmap::IndexMap::new();
        nodes.insert(
            "/ctrl".to_string(),
            NodeInstance {
                scope: "/".to_string(),
                criticality: Some("high".to_string()),
                ..Default::default()
            },
        );
        let mut node_paths = BTreeMap::new();
        // Periodic 100 Hz control path with a 5 ms deadline.
        node_paths.insert(
            "/ctrl/loop".to_string(),
            PathContract {
                input: vec![],
                output: vec!["/ctrl/cmd".to_string()],
                max_latency_ms: Some(5.0),
                ..Default::default()
            },
        );
        let mut pub_endpoints = BTreeMap::new();
        pub_endpoints.insert(
            "/ctrl/cmd".to_string(),
            ros_launch_manifest_model::PubContract {
                min_rate_hz: Some(100.0),
                ..Default::default()
            },
        );
        SystemModel {
            structure: Structure {
                nodes,
                ..Default::default()
            },
            contracts: Contracts {
                node_paths,
                pub_endpoints,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    fn ctrl_groups() -> BTreeMap<String, Vec<CallbackGroupDecl>> {
        let mut groups = BTreeMap::new();
        groups.insert(
            "ctrl".to_string(),
            vec![CallbackGroupDecl {
                id: "main".to_string(),
                r#type: "MutuallyExclusive".to_string(),
                tier: "default".to_string(),
            }],
        );
        groups
    }

    #[test]
    fn derives_tiers_from_contracts_when_none_declared() {
        let mut system: SystemToml =
            toml::from_str("[system]\nname = \"t\"\nrmw = \"zenoh\"\ndomain_id = 0\n").unwrap();
        let model = contract_model();

        let n = derive_execution_from_contracts(&mut system, &model, "zephyr", &ctrl_groups())
            .expect("derivation succeeds");
        assert_eq!(n.0, 1, "one contracted node → one derived tier");
        // Issue 0259 — a well-formed contracted node with no declared WCETs
        // produces no verdicts: the realizer has nothing to judge.
        assert!(n.1.is_empty(), "unexpected verdicts: {:?}", n.1);

        let tier = system
            .tiers
            .get("derived-ctrl")
            .expect("derived tier present");
        assert_eq!(
            tier.class.as_deref(),
            Some("real_time"),
            "deadline ⇒ real_time"
        );
        assert_eq!(tier.deadline_us, Some(5_000));
        assert_eq!(tier.period_us, Some(10_000), "100 Hz → 10 ms period");
        let z = tier
            .zephyr
            .as_ref()
            .expect("zephyr sub-table on a zephyr bake");
        assert!(
            z.sched_class.is_none(),
            "generic policy carries the semantics"
        );

        // Binding: the node's declared group is reassigned to the derived tier.
        assert_eq!(system.node_overrides.len(), 1);
        assert_eq!(system.node_overrides[0].name, "ctrl");
        assert_eq!(
            system.node_overrides[0].callback_groups[0].tier,
            "derived-ctrl"
        );
    }

    #[test]
    fn groupless_node_stays_on_default_tier() {
        let mut system: SystemToml =
            toml::from_str("[system]\nname = \"t\"\nrmw = \"zenoh\"\ndomain_id = 0\n").unwrap();
        let model = contract_model();
        // No callback groups declared → nothing to bind → no derived tier.
        let n = derive_execution_from_contracts(&mut system, &model, "zephyr", &BTreeMap::new())
            .expect("derivation succeeds");
        assert_eq!(n.0, 0);
        assert!(system.tiers.is_empty());
    }

    /// phase-296 W5.15 — edf knobs on entries for DIFFERENT boards must NOT
    /// force a bake to agree: derive runs once per target_rtos, and each image
    /// has its own kernel. A zephyr entry (edf=true) + a freertos entry
    /// (edf=false) is a legal mixed model — each bake slices to its own board.
    #[test]
    fn edf_knobs_sliced_per_target_rtos() {
        use ros_launch_manifest_model::{Deploy, ExtraValue, Target};
        let make = |target_rtos: &str| {
            let mut system: SystemToml =
                toml::from_str("[system]\nname = \"t\"\nrmw = \"zenoh\"\ndomain_id = 0\n").unwrap();
            let mut model = contract_model();
            for (node, board, edf) in [
                ("/ctrl", "zephyr-native-sim", true),
                ("/telem", "mps2-an385-freertos", false),
            ] {
                let mut extra = BTreeMap::new();
                extra.insert("edf".to_string(), ExtraValue::Bool(edf));
                model.execution.deploy.insert(
                    node.to_string(),
                    Deploy {
                        target: Some(Target::Mcu {
                            board: board.to_string(),
                        }),
                        extra,
                        ..Default::default()
                    },
                );
            }
            derive_execution_from_contracts(&mut system, &model, target_rtos, &ctrl_groups())
        };
        // Before W5.15 both bakes bailed on the cross-board disagreement.
        assert!(
            make("zephyr").is_ok(),
            "zephyr bake must slice to the zephyr entry's edf=true, not bail on the \
             freertos entry"
        );
        assert!(
            make("freertos").is_ok(),
            "freertos bake must slice to the freertos entry's edf=false"
        );
    }
}

/// R1-N1 — one contracted-publisher monitor row extracted from the model
/// (RFC-0052 W3b.4 consumer side).
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct MonitorRow {
    /// Topic FQN (the wiring name the publisher creates).
    pub topic: String,
    /// Endpoint ref (`<node FQN>/<endpoint>`), the violation report key.
    pub fqn: String,
    /// Declared publisher guarantee, milli-Hz. 0 = no rate contract
    /// (latency-only row).
    pub min_rate_hz_milli: u32,
    /// W3b.5 — node-path budget (ms) for paths whose output is this
    /// endpoint (`contracts.node_paths`). 0 = no latency contract.
    #[serde(skip_serializing_if = "is_zero", default)]
    pub max_latency_ms: u32,
}

fn is_zero(v: &u32) -> bool {
    *v == 0
}

/// W3b.5 — one contracted-subscriber age row (`sub_endpoints.max_age_ms`).
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct AgeRow {
    /// Topic FQN (the wiring name the subscriber creates).
    pub topic: String,
    /// Endpoint ref, the violation report key.
    pub fqn: String,
    /// Declared max take-age, ms.
    pub max_age_ms: u32,
}

/// Extract the publisher rate-monitor rows: every `pub_endpoints` entry
/// with `min_rate_hz`, joined to the topic whose wiring lists it as a
/// publisher. A contracted endpoint with NO owning topic in the wiring is
/// a model inconsistency — fail loud.
pub fn monitor_rows(model: &SystemModel) -> Result<Vec<MonitorRow>> {
    use std::collections::BTreeMap;
    // fqn -> (min_rate_milli, max_latency_ms); node_paths may add
    // latency-only rows for endpoints without a rate contract.
    let mut by_fqn: BTreeMap<String, (u32, u32)> = BTreeMap::new();
    for (ep_ref, c) in &model.contracts.pub_endpoints {
        let Some(min) = c.min_rate_hz else { continue };
        by_fqn.insert(
            ep_ref.clone(),
            (
                (min * 1000.0).round().max(0.0).min(u32::MAX as f64) as u32,
                0,
            ),
        );
    }
    // W3b.5 — node-path budgets attach to the path's OUTPUT endpoints.
    for (path_ref, p) in &model.contracts.node_paths {
        let Some(lat) = p.max_latency_ms else {
            continue;
        };
        let lat = lat.round().max(0.0).min(u32::MAX as f64) as u32;
        if lat == 0 {
            continue;
        }
        if p.output.is_empty() {
            bail!(
                "SystemModel: node path '{path_ref}' declares max_latency_ms but \
                 lists no output endpoint — inconsistent model"
            );
        }
        for out in &p.output {
            let e = by_fqn.entry(out.clone()).or_insert((0, 0));
            e.1 = e.1.max(lat);
        }
    }
    let mut rows = Vec::new();
    for (ep_ref, (min_milli, lat_ms)) in by_fqn {
        let topic = model
            .structure
            .topics
            .iter()
            .find(|(_, w)| w.publishers.iter().any(|p| p == &ep_ref))
            .map(|(t, _)| t.clone());
        let Some(topic) = topic else {
            bail!(
                "SystemModel: contracted publisher '{ep_ref}' has no \
                 owning topic in structure.topics — inconsistent model"
            );
        };
        rows.push(MonitorRow {
            topic,
            fqn: ep_ref,
            min_rate_hz_milli: min_milli,
            max_latency_ms: lat_ms,
        });
    }
    Ok(rows)
}

/// W3b.5 — extract the subscriber age rows: every `sub_endpoints` entry
/// with `max_age_ms`, joined to the topic whose wiring lists it as a
/// subscriber. Orphans fail loud (same rule as the publisher join).
pub fn age_rows(model: &SystemModel) -> Result<Vec<AgeRow>> {
    let mut rows = Vec::new();
    for (ep_ref, c) in &model.contracts.sub_endpoints {
        let Some(age) = c.max_age_ms else { continue };
        let topic = model
            .structure
            .topics
            .iter()
            .find(|(_, w)| w.subscribers.iter().any(|p| p == ep_ref))
            .map(|(t, _)| t.clone());
        let Some(topic) = topic else {
            bail!(
                "SystemModel: contracted subscriber '{ep_ref}' (max_age_ms) has no \
                 owning topic in structure.topics — inconsistent model"
            );
        };
        rows.push(AgeRow {
            topic,
            fqn: ep_ref.clone(),
            max_age_ms: age.round().max(1.0).min(u32::MAX as f64) as u32,
        });
    }
    rows.sort_by(|a, b| a.fqn.cmp(&b.fqn));
    Ok(rows)
}

/// R1-N1 — render the baked Rust monitor-table include
/// (`nros-system/system_monitors.rs`): one `PubMonitorCell` static per
/// contracted publisher + the `MONITORS` spec table + an installer the
/// generated entry calls before entity creation. Empty rows → an empty
/// table (DCE'd — the zero-cost gate).
pub fn render_monitor_rs(rows: &[MonitorRow], ages: &[AgeRow]) -> String {
    let mut out = String::new();
    out.push_str(
        "// GENERATED by `nros codegen-system --model` (RFC-0052 W3b.4/.5 / phase-296 N1).\n\
         // One PubMonitorCell per contracted publisher (+ one SubMonitorCell per age\n\
         // contract) + the executor monitor tables. Include from the entry; call\n\
         // `nros_install_monitors(&mut executor)` BEFORE entity creation (node-side\n\
         // attachment is auto-seeded from the executor at create_node).\n\
         use ::nros_node::executor::monitor::{AgeMonitorSpec, MonitorSpec, PubMonitorCell, SubMonitorCell};\n\n",
    );
    for (i, _r) in rows.iter().enumerate() {
        out.push_str(&format!(
            "static NROS_MONITOR_CELL_{i}: PubMonitorCell = PubMonitorCell::new();\n"
        ));
    }
    for (i, _r) in ages.iter().enumerate() {
        out.push_str(&format!(
            "static NROS_AGE_CELL_{i}: SubMonitorCell = SubMonitorCell::new();\n"
        ));
    }
    out.push_str("\npub static NROS_MONITORS: &[MonitorSpec] = &[\n");
    for (i, r) in rows.iter().enumerate() {
        out.push_str(&format!(
            "    MonitorSpec {{ topic: {t:?}, fqn: {f:?}, min_rate_hz_milli: {m}u32, \
             max_latency_ms: {l}u32, cell: &NROS_MONITOR_CELL_{i} }},\n",
            t = r.topic,
            f = r.fqn,
            m = r.min_rate_hz_milli,
            l = r.max_latency_ms,
        ));
    }
    out.push_str("];\n\n");
    out.push_str("pub static NROS_AGE_MONITORS: &[AgeMonitorSpec] = &[\n");
    for (i, r) in ages.iter().enumerate() {
        out.push_str(&format!(
            "    AgeMonitorSpec {{ topic: {t:?}, fqn: {f:?}, max_age_ms: {a}u32, \
             cell: &NROS_AGE_CELL_{i} }},\n",
            t = r.topic,
            f = r.fqn,
            a = r.max_age_ms,
        ));
    }
    out.push_str("];\n\n");
    out.push_str(
        "pub fn nros_install_monitors(executor: &mut ::nros_node::executor::Executor<'_>) {\n    executor.set_monitor_table(NROS_MONITORS);\n    executor.set_age_table(NROS_AGE_MONITORS);\n}\n",
    );
    out
}

#[cfg(test)]
mod monitor_tests {
    use super::*;
    use ros_launch_manifest_model::{PubContract, TopicWiring};

    fn model_with_contract() -> SystemModel {
        let mut m = SystemModel::default();
        m.structure.topics.insert(
            "/perception/objects".to_string(),
            TopicWiring {
                msg_type: "std_msgs/msg/String".to_string(),
                publishers: vec!["/perception/detector/objects".to_string()],
                subscribers: vec![],
            },
        );
        m.contracts.pub_endpoints.insert(
            "/perception/detector/objects".to_string(),
            PubContract {
                min_rate_hz: Some(10.0),
                ..Default::default()
            },
        );
        m
    }

    #[test]
    fn rows_join_endpoint_to_topic_and_render() {
        let rows = monitor_rows(&model_with_contract()).expect("rows");
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].topic, "/perception/objects");
        assert_eq!(rows[0].min_rate_hz_milli, 10_000);
        let rs = render_monitor_rs(&rows, &[]);
        assert!(rs.contains("NROS_MONITOR_CELL_0"));
        assert!(rs.contains("min_rate_hz_milli: 10000u32"));
        assert!(rs.contains("topic: \"/perception/objects\""));
    }

    #[test]
    fn orphan_contract_fails_loud() {
        let mut m = model_with_contract();
        m.structure.topics.clear();
        let err = monitor_rows(&m).unwrap_err().to_string();
        assert!(
            err.contains("no \\\n                 owning topic") || err.contains("no owning topic"),
            "{err}"
        );
    }

    #[test]
    fn empty_contracts_render_empty_table() {
        let rows = monitor_rows(&SystemModel::default()).expect("rows");
        assert!(rows.is_empty());
        let rs = render_monitor_rs(&rows, &[]);
        assert!(rs.contains("NROS_MONITORS: &[MonitorSpec] = &[\n];"));
        assert!(rs.contains("NROS_AGE_MONITORS: &[AgeMonitorSpec] = &[\n];"));
    }

    #[test]
    fn age_rows_join_subscriber_and_render() {
        use ros_launch_manifest_model::SubContract;
        let mut m = SystemModel::default();
        m.structure.topics.insert(
            "/sensing/scan".to_string(),
            TopicWiring {
                msg_type: "sensor_msgs/msg/LaserScan".to_string(),
                publishers: vec![],
                subscribers: vec!["/perception/detector/scan".to_string()],
            },
        );
        m.contracts.sub_endpoints.insert(
            "/perception/detector/scan".to_string(),
            SubContract {
                max_age_ms: Some(100.0),
                ..Default::default()
            },
        );
        let ages = age_rows(&m).expect("ages");
        assert_eq!(ages.len(), 1);
        assert_eq!(ages[0].topic, "/sensing/scan");
        assert_eq!(ages[0].max_age_ms, 100);
        let rs = render_monitor_rs(&[], &ages);
        assert!(rs.contains("NROS_AGE_CELL_0"));
        assert!(rs.contains("max_age_ms: 100u32"));
        assert!(rs.contains("set_age_table(NROS_AGE_MONITORS)"));

        // Orphan sub contract: fail loud.
        m.structure.topics.clear();
        let err = age_rows(&m).unwrap_err().to_string();
        assert!(
            err.contains("no owning topic") || err.contains("owning topic"),
            "{err}"
        );
    }

    #[test]
    fn node_path_budget_attaches_to_output_endpoint() {
        use ros_launch_manifest_model::PathContract;
        let mut m = model_with_contract();
        m.contracts.node_paths.insert(
            "/perception/detector/proc".to_string(),
            PathContract {
                input: vec!["/perception/detector/scan".to_string()],
                output: vec!["/perception/detector/objects".to_string()],
                max_latency_ms: Some(20.0),
                ..Default::default()
            },
        );
        let rows = monitor_rows(&m).expect("rows");
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].max_latency_ms, 20);
        assert_eq!(rows[0].min_rate_hz_milli, 10_000, "rate contract kept");
        let rs = render_monitor_rs(&rows, &[]);
        assert!(rs.contains("max_latency_ms: 20u32"));
    }
}

/// R1-N3 — convert the model's transport declarations into the plan's
/// [`PlanTransport`] shape (the type the board network bake consumes).
/// Unknown `kind` strings are a bake-time error (fail-loud).
/// R-code plan rework — synthesize the play_launch record shape the
/// workspace planner consumes (`{node: [...], container: [], load_node: []}`)
/// from a resolved SystemModel, so `nros plan` can plan from the committed
/// model without parsing launch XML. Params/remaps/env come resolved from
/// the model (post-46 `NodeInstance` carries them all).
pub fn plan_record_from_model(model: &SystemModel) -> serde_json::Value {
    let mut nodes: Vec<serde_json::Value> = Vec::new();
    let mut containers: Vec<serde_json::Value> = Vec::new();
    let mut load_nodes: Vec<serde_json::Value> = Vec::new();
    for (fqn, inst) in &model.structure.nodes {
        let bare = fqn.rsplit('/').next().unwrap_or(fqn).to_string();
        let ns = {
            let ns = &fqn[..fqn.len() - bare.len()];
            let ns = ns.trim_end_matches('/');
            if ns.is_empty() {
                "/".to_string()
            } else {
                ns.to_string()
            }
        };
        // #276 — projected params (files under inline).
        let params: Vec<serde_json::Value> = inst
            .resolved_params(fqn)
            .iter()
            .map(|(k, v)| serde_json::json!([k, v.to_bake_string()]))
            .collect();
        let remaps: Vec<serde_json::Value> = inst
            .remaps
            .iter()
            .map(|r| serde_json::json!([r.from, r.to]))
            .collect();
        let env: Vec<serde_json::Value> = inst
            .env
            .iter()
            .map(|e| serde_json::json!([e.name, e.value]))
            .collect();
        // Three-way classification (launch_synth-deletion precondition #1):
        // a container node feeds the record's `container` array, a composable
        // child (plugin + hosting container FQN) feeds `load_node` (linked by
        // `target_container_name`, the parser's shape the planner already
        // resolves), everything else is a plain `node`.
        if inst.is_container {
            containers.push(serde_json::json!({
                "package": inst.pkg.clone().unwrap_or_default(),
                "executable": inst.exec.clone().unwrap_or_default(),
                "name": bare,
                "namespace": ns,
                "params": params,
                "remaps": remaps,
                "env": env,
            }));
        } else if let (Some(plugin), Some(target)) = (&inst.plugin, &inst.container) {
            load_nodes.push(serde_json::json!({
                "package": inst.pkg.clone().unwrap_or_default(),
                "plugin": plugin,
                "target_container_name": target,
                "node_name": bare,
                "namespace": ns,
                "params": params,
                "remaps": remaps,
                "env": env,
            }));
        } else {
            nodes.push(serde_json::json!({
                "package": inst.pkg.clone().unwrap_or_default(),
                "executable": inst
                    .exec
                    .clone()
                    .or_else(|| {
                        inst.plugin
                            .as_deref()
                            .map(|p| p.rsplit("::").next().unwrap_or(p).to_string())
                    })
                    .unwrap_or_default(),
                "name": bare,
                "namespace": ns,
                "params": params,
                "remaps": remaps,
                "env": env,
            }));
        }
    }
    // Phase 434 — the exclusion relation rides the record, because the
    // planner infers callback groups from a record and never sees the model.
    // The KEY is always present when the record came from a model, even when
    // no node declares anything: its presence is what tells the planner the
    // contract's default applies (everything serialises), as opposed to a
    // legacy record where the planner's own inference is all there is.
    let node_concurrency: serde_json::Map<String, serde_json::Value> = model
        .contracts
        .node_concurrency
        .iter()
        .map(|(fqn, c)| (fqn.clone(), serde_json::json!(c.exclusive)))
        .collect();
    serde_json::json!({
        "node": nodes,
        "container": containers,
        "load_node": load_nodes,
        "node_concurrency": node_concurrency,
    })
}

pub fn plan_transports(
    model: &SystemModel,
) -> Result<Vec<crate::orchestration::plan::PlanTransport>> {
    use crate::orchestration::plan::{PlanTransport, TransportKind};
    let mut out = Vec::new();
    for t in &model.execution.transports {
        let kind = match t.kind.as_str() {
            "ethernet" => TransportKind::Ethernet,
            "wifi" => TransportKind::Wifi,
            "serial" => TransportKind::Serial,
            "can" => TransportKind::Can,
            other => bail!(
                "SystemModel transport kind '{other}' is not supported \
                 (ethernet | wifi | serial | can)"
            ),
        };
        // phase-206 W5 — `TransportBlock.interfaces` lives in the UPSTREAM
        // `ros-launch-manifest` schema, which we do not own, so the key can
        // still arrive from a `system.toml`. nano-ros has no NIC vocabulary to
        // lower it into any more; refuse it here rather than dropping it
        // silently, which is strictly worse than the no-op it used to be.
        if !t.interfaces.is_empty() {
            bail!(
                "SystemModel transport '{}': {}",
                t.id.as_deref().unwrap_or(&t.kind),
                crate::orchestration::plan::RETIRED_INTERFACES_REMEDY
            );
        }
        out.push(PlanTransport {
            kind,
            id: t.id.clone(),
            ip: t.ip.clone(),
            ssid: t.ssid.clone(),
            password: t.password.clone(),
            mac: t.mac.clone(),
            gateway: t.gateway.clone(),
            device: t.device.clone(),
            baudrate: t.baudrate,
            rmw: t.rmw.clone(),
            locator: t.locator.clone(),
            domain: t.domain.map(u32::from),
        });
    }
    Ok(out)
}

#[cfg(test)]
mod transport_tests {
    use super::*;
    use ros_launch_manifest_model::Transport;

    #[test]
    fn transports_convert_and_unknown_kind_fails() {
        let mut m = SystemModel::default();
        m.execution.transports.push(Transport {
            kind: "ethernet".to_string(),
            id: Some("eth0".to_string()),
            ip: Some("10.0.2.50/24".to_string()),
            mac: Some("02:00:00:00:00:01".to_string()),
            domain: Some(7),
            ..Default::default()
        });
        let pts = plan_transports(&m).expect("converts");
        assert_eq!(pts.len(), 1);
        assert_eq!(pts[0].mac.as_deref(), Some("02:00:00:00:00:01"));
        assert_eq!(pts[0].domain, Some(7));

        m.execution.transports[0].kind = "carrier-pigeon".to_string();
        assert!(plan_transports(&m).is_err());
    }

    /// phase-206 W5 — the upstream schema still HAS `interfaces`, so the only
    /// thing under our control is what we do when it arrives. Refuse, naming
    /// the backend config that does own NIC binding; the pre-W5 behaviour was
    /// to copy it into a plan nothing read.
    #[test]
    fn a_transport_naming_interfaces_is_refused_with_the_backend_remedy() {
        let mut m = SystemModel::default();
        m.execution.transports.push(Transport {
            kind: "ethernet".to_string(),
            id: Some("eth0".to_string()),
            interfaces: vec!["eth0".to_string(), "eth1".to_string()],
            ..Default::default()
        });
        let err = plan_transports(&m)
            .expect_err("an `interfaces` list must fail the bake")
            .to_string();
        assert!(err.contains("eth0"), "names the transport: {err}");
        assert!(err.contains("CYCLONEDDS_URI"), "names the remedy: {err}");
    }
}

#[cfg(test)]
mod cyclonedds_type_capacity_tests {
    //! Issue 0284 — model-derived CycloneDDS type-registry sizing (resolve + emit).
    use ros_launch_manifest_model::{ServiceWiring, SystemModel, TopicWiring};

    use super::{manage_cyclonedds_max_types, resolve_cyclonedds_max_types_with};

    /// A model with `msgs` distinct message topics + `actions` distinct actions.
    fn model(msgs: usize, actions: usize) -> SystemModel {
        let mut m = SystemModel::default();
        for i in 0..msgs {
            m.structure.topics.insert(
                format!("/t{i}"),
                TopicWiring {
                    msg_type: format!("pkg/msg/M{i}"),
                    publishers: vec![format!("/n/t{i}")],
                    subscribers: vec![],
                },
            );
        }
        for i in 0..actions {
            m.structure.actions.insert(
                format!("/a{i}"),
                ServiceWiring {
                    srv_type: format!("pkg/action/A{i}"),
                    server: vec![format!("/n/a{i}")],
                    client: vec![],
                },
            );
        }
        m
    }

    #[test]
    fn empty_and_small_models_emit_nothing() {
        assert_eq!(
            resolve_cyclonedds_max_types_with(&SystemModel::default(), None).unwrap(),
            None
        );
        // 10 msgs = 10 types <= default 32 → keep default, no emit.
        assert_eq!(
            resolve_cyclonedds_max_types_with(&model(10, 0), None).unwrap(),
            None
        );
    }

    #[test]
    fn large_model_auto_sizes_to_next_power_of_two() {
        // 40 distinct msgs = 40 types → next_pow2 = 64.
        assert_eq!(
            resolve_cyclonedds_max_types_with(&model(40, 0), None).unwrap(),
            Some(64)
        );
        // 4 actions = 4*8 + 3 = 35 types → 64.
        assert_eq!(
            resolve_cyclonedds_max_types_with(&model(0, 4), None).unwrap(),
            Some(64)
        );
    }

    #[test]
    fn user_pin_too_small_fails_loud() {
        // 4 actions = 35 types, pinned to 32 → bail naming the derived 64.
        let err = resolve_cyclonedds_max_types_with(&model(0, 4), Some(32))
            .unwrap_err()
            .to_string();
        assert!(err.contains("35 distinct DDS types"), "{err}");
        assert!(err.contains("64"), "{err}");
        assert!(err.contains("NROS_CYCLONEDDS_MAX_TYPES"), "{err}");
    }

    #[test]
    fn user_pin_large_enough_is_respected_no_emit() {
        assert_eq!(
            resolve_cyclonedds_max_types_with(&model(0, 4), Some(128)).unwrap(),
            None
        );
    }

    #[test]
    fn emit_writes_env_with_force_and_tag_then_removes_idempotently() {
        let tmp = tempfile::tempdir().unwrap();
        let ws = tmp.path();

        // Insert.
        assert!(manage_cyclonedds_max_types(ws, Some(64)).unwrap());
        let cfg = std::fs::read_to_string(ws.join(".cargo/config.toml")).unwrap();
        assert!(cfg.contains("[env]"), "{cfg}");
        assert!(cfg.contains("NROS_CYCLONEDDS_MAX_TYPES"), "{cfg}");
        assert!(cfg.contains("value = \"64\""), "{cfg}");
        assert!(cfg.contains("force = true"), "{cfg}");
        assert!(cfg.contains("nros-managed"), "{cfg}");

        // Idempotent: same value → no change.
        assert!(!manage_cyclonedds_max_types(ws, Some(64)).unwrap());

        // Remove managed line when the model shrinks back under the default.
        assert!(manage_cyclonedds_max_types(ws, None).unwrap());
        let cfg = std::fs::read_to_string(ws.join(".cargo/config.toml")).unwrap();
        assert!(!cfg.contains("NROS_CYCLONEDDS_MAX_TYPES"), "{cfg}");
    }

    #[test]
    fn a_users_unmanaged_env_line_is_never_clobbered() {
        let tmp = tempfile::tempdir().unwrap();
        let ws = tmp.path();
        std::fs::create_dir_all(ws.join(".cargo")).unwrap();
        std::fs::write(
            ws.join(".cargo/config.toml"),
            "[env]\nNROS_CYCLONEDDS_MAX_TYPES = \"256\"\n",
        )
        .unwrap();

        // Emit is skipped (no tag) — the user's value stays authoritative.
        assert!(!manage_cyclonedds_max_types(ws, Some(64)).unwrap());
        let cfg = std::fs::read_to_string(ws.join(".cargo/config.toml")).unwrap();
        assert!(cfg.contains("\"256\""), "{cfg}");
        // Removal also leaves an unmanaged line alone.
        assert!(!manage_cyclonedds_max_types(ws, None).unwrap());
        assert!(
            std::fs::read_to_string(ws.join(".cargo/config.toml"))
                .unwrap()
                .contains("\"256\"")
        );
    }
}
