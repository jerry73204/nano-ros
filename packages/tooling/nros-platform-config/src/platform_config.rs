//! RFC-0049 / phase-290 — per-package hierarchical platform/board
//! configuration.
//!
//! One `nros-platform.toml` per platform package directory
//! (`config/<name>/`, or an out-of-tree dir named via
//! `NROS_PLATFORMS_DIR` + the platform name), carrying:
//!
//! ```toml
//! inherits = "generic"        # optional family chain (sibling dir name)
//!
//! [capabilities]              # software-stack FACTS (open vocabulary)
//! threads = true
//! per_fd_tx_ceiling = true
//!
//! [knobs.zenoh.tx]            # policy defaults (typed, deny_unknown_fields)
//! batch = true
//! split_lock = true
//! flush_ms = 50
//!
//! [build.zenoh]               # the former zenoh_platforms.toml block,
//! defines = ["ZENOH_GENERIC"] # keys verbatim (RFC-0049 open question 1:
//! # ...                       # verbatim relocation)
//!
//! [arch.cortex-m3]            # reusable compiler-flag profiles; may be
//! # ...                       # duplicated across files if byte-identical
//! ```
//!
//! Board packages carry the same `[capabilities]` / `[knobs.*]` tables in
//! their existing `nros-board.toml` (RFC-0042 descriptor) as deltas.
//!
//! Resolution ladder (RFC-0004 style — fixed, not an open merge):
//!
//! ```text
//! built-in default < platform file(s, via inherits) < board file < env
//! ```
//!
//! Env front-ends are tri-state: unset = defer to the ladder below; set
//! (including explicit `0`) = override. Every resolved knob remembers which
//! rung set it (`KnobSource`) so `nros config explain` can print the ladder.
//!
//! The schema home is this crate rather than `nros-platform` (the RFC's
//! first draft): `nros-platform` is a `no_std` runtime crate, while this
//! module is build-time tooling next to the existing manifest parser it
//! builds on.

use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

use serde::Deserialize;

use crate::manifest::{ArchEntry, ManifestError, PlatformEntry, PlatformManifest};

/// Filename of the per-platform-package config file.
pub const PLATFORM_CONFIG_FILENAME: &str = "nros-platform.toml";

/// One `nros-platform.toml` file, parsed.
///
/// Every section is optional — an absent/empty file is valid and yields
/// pure built-in behavior (the byte-identity guarantee phase-290 W2.c
/// regression-tests).
#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlatformConfigFile {
    /// phase-349 W1 / RFC-0072 — the names this platform answers to.
    ///
    /// Empty (the default) means "just my directory name", which is what every
    /// file meant before this existed. The first entry is canonical; the rest
    /// are aliases, which is how `freertos-lwip` keeps resolving after the
    /// directory became `freertos`.
    ///
    /// The stack does not belong in a platform's identity — zenoh-pico itself
    /// splits `system/freertos/{system.c, lwip/network.c,
    /// freertos_plus_tcp/network.c}` — so `freertos-lwip` is an alias to retire,
    /// not a name to keep. Matching the rmw and board descriptors also lets
    /// `check-provider-announcements.py` compare provisions against it with one
    /// more `FAMILIES` row rather than a new rule.
    #[serde(default)]
    pub names: Vec<String>,

    /// Optional parent platform (sibling directory name). The parent's
    /// `[build.zenoh]`, `[capabilities]` and `[knobs]` merge underneath
    /// this file's values.
    #[serde(default)]
    pub inherits: Option<String>,
    /// Software-stack facts. Open vocabulary by design — facts are
    /// consumed by name in capability checks; policy (knobs) is the
    /// typed, closed part of the schema.
    #[serde(default)]
    pub capabilities: BTreeMap<String, bool>,
    /// `[priority_plan]` — ACKNOWLEDGED here, interpreted elsewhere.
    ///
    /// phase-375 W8 gave RFC-0079's priority address plan a per-platform home
    /// (`config/freertos/nros-platform.toml`), and its readers are
    /// `scripts/lib/priority_plan.py` and its two consumers — not this crate.
    ///
    /// Modelled as an OPAQUE value on purpose, the same way
    /// `BoardDescriptor::priority_plan` is: `deny_unknown_fields` above must
    /// not come to mean "every consumer's schema is restated here", which would
    /// make this struct the union of several readers and guarantee drift.
    /// Declaring it says "this key is real and someone else owns it"; omitting
    /// it said "typo", and that is exactly what happened — adding the table
    /// broke every embedded build with `unknown field `priority_plan``, in a
    /// build script, where the fast lane cannot see it because it does not
    /// compile.
    #[serde(default)]
    pub priority_plan: Option<toml::Value>,
    #[serde(default)]
    pub knobs: Knobs,
    #[serde(default)]
    pub build: BuildSection,
    /// Reusable compiler-flag profiles. Files may repeat a profile
    /// (e.g. `cortex-m3` in both `bare-metal` and `freertos-lwip`)
    /// only if the copies are identical; conflicting redefinition is a
    /// load error.
    #[serde(default)]
    pub arch: BTreeMap<String, ArchEntry>,
}

/// `[build.*]` — per-vendored-component build blocks, keyed by COMPONENT NAME.
///
/// phase-347 W6 — this was `struct BuildSection { zenoh: Option<PlatformEntry> }`
/// with `deny_unknown_fields`, so `[build.cyclonedds]` was not merely absent, it
/// was REJECTED: a platform could describe exactly one backend's vendored C
/// build, and the one it could describe was named in core.
///
/// Only the KEY was ever backend-specific. `PlatformEntry` carries `defines`,
/// `include`, `extra_sources`, `arch`, `compile` … — generic vendored-library
/// build config with no zenoh-shaped field, already proven across the seven
/// `config/*/nros-platform.toml` files. So this is a keying change, not a schema
/// design: `[build.zenoh]` parses as the key `"zenoh"` and **none of those seven
/// files change**.
///
/// The second tenant is not hypothetical: `nros-rmw-xrce-cffi/build.rs` is ~500
/// lines hardcoding this same shape (`_DEFAULT_SOURCE`, `_POSIX_C_SOURCE`,
/// posix/embedded branching, a generated config header) because there was
/// nowhere to declare it.
pub type BuildSection = BTreeMap<String, PlatformEntry>;

/// `[knobs]` — typed policy. `zenoh.tx` is the first tenant
/// (phase-282); future tenants (`executor`, `log`, ring depths, …) are
/// additive fields here.
/// `[knobs]` — typed policy.
///
/// phase-400 W2 / RFC-0071 D8. `transport` is a RESERVED, cross-cutting key;
/// every other key is a BACKEND NAME, so a platform can say "here are my
/// settings for whichever backend is selected" without naming one in its
/// schema. `[build.<rmw>]` was de-keyed the same way by phase-347 W6; this is
/// the knobs half of the same rule.
///
/// `deny_unknown_fields` is dropped here and NOWHERE else, because serde does
/// not support it alongside `flatten`. The validation it provided is not lost:
/// [`Knobs::unknown_backend_keys`] reports a key that no backend descriptor
/// claims, which is the diagnostic RFC-0049 asks for ("unknown keys fail loud
/// with the valid-key list") and is strictly better here — a backend name is
/// only knowable once the descriptors are loaded, which serde cannot do.
#[derive(Debug, Default, Clone, Deserialize)]
pub struct Knobs {
    /// Backwards compatibility: `[knobs.zenoh.tx]` in the seven in-tree files
    /// still parses through this field, so no platform file changes. New
    /// backends use the flattened map below.
    #[serde(default)]
    pub zenoh: ZenohKnobs,
    /// phase-400 W6 — the executor sizing tenant.
    ///
    /// Eight knobs that `nros-node/build.rs` reads straight from the
    /// environment, so today they reach a build only from the shell that ran
    /// it: a bare `ninja` in a configured tree silently rebuilds at crate
    /// defaults (the failure nano-ros #0749/#0752 fixed for the Zephyr lane by
    /// adding Kconfig rows, and which every other lane still has).
    ///
    /// Migrating them here does not change a single call site — each keeps its
    /// existing env name as the lane front-end — but it gives them a platform
    /// and board rung, and makes them visible to `nros config explain`.
    #[serde(default)]
    pub executor: ExecutorKnobs,
    /// phase-400 W6 — the platform memory tenant (RTOS heap, app stack).
    #[serde(default)]
    pub memory: MemoryKnobs,
    /// phase-400 W6 — the parameter-storage tenant.
    #[serde(default)]
    pub params: ParamKnobs,
    /// phase-400 W6 — the RMW static-pool tenant. See [`RmwKnobs`].
    #[serde(default)]
    pub rmw: RmwKnobs,
    /// phase-400 W6 — the smoltcp net tenant. See [`NetKnobs`].
    #[serde(default)]
    pub net: NetKnobs,
    /// phase-400 W6 — the component-runtime tenant. See [`RuntimeKnobs`].
    #[serde(default)]
    pub runtime: RuntimeKnobs,
    /// phase-400 W6 — the XRCE transport tenant. See [`XrceKnobs`].
    #[serde(default)]
    pub xrce: XrceKnobs,
    /// phase-400 W3 / RFC-0086 D1 — the transport tenant.
    ///
    /// The first knob that crosses all three descriptor axes: the backend knows
    /// how to SPELL an endpoint, the platform knows whether an IP stack EXISTS,
    /// the board knows which peripheral is WIRED. Stating it once is what lets
    /// the resolver derive the link and driver settings that a transport choice
    /// implies, instead of each image hand-writing them.
    #[serde(default)]
    pub transport: TransportKnobs,
    /// Any other key is a backend name. Empty for every in-tree file today,
    /// which is the point: a third-party RMW fills it without nano-ros
    /// learning its name.
    #[serde(flatten, default)]
    pub backends: BTreeMap<String, BackendKnobs>,
}

/// Per-backend policy, keyed by the backend's own name in `[knobs.<rmw>]`.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BackendKnobs {
    #[serde(default)]
    pub tx: TxKnobs,
}

impl Knobs {
    /// Keys under `[knobs]` that are neither the reserved `transport` tenant
    /// nor a backend the caller knows about. Caller supplies the known set,
    /// because only it has loaded the descriptors.
    pub fn unknown_backend_keys(&self, known: &[&str]) -> Vec<String> {
        self.backends
            .keys()
            .filter(|k| !known.contains(&k.as_str()))
            .cloned()
            .collect()
    }

    /// The `tx` knobs for `backend`, preferring the flattened per-backend
    /// table and falling back to the legacy `[knobs.zenoh.tx]` field.
    pub fn tx_for(&self, backend: &str) -> &TxKnobs {
        match self.backends.get(backend) {
            Some(b) => &b.tx,
            None => &self.zenoh.tx,
        }
    }
}

/// Every executor knob, in a stable order.
///
/// One list, so a caller that needs to go the other way (env name → knob) can
/// derive it from [`executor_env_key`] instead of retyping the pairs. Two
/// hand-maintained copies of "which knobs exist" is how one of them ends up
/// missing the knob that was just added.
pub const EXECUTOR_KNOBS: &[&str] = &[
    "max_cbs",
    "max_sc",
    "max_nodes",
    "max_shutdown_cbs",
    "action_clients",
    "arena_size",
    "subscription_buffer_size",
    "param_service_buffer_size",
];

/// The platform and board rungs a BUILD SCRIPT can reach, resolved once.
///
/// phase-400 W6. A build script learns its platform and board the way every
/// other build-time fact travels here: the lane exports a value and a POINTER,
/// and the script reads the file. `nros ws board-facts` emits
/// `NROS_PLATFORM_NAME` and `NROS_BOARD_TOML`, and `corrosion_set_env_vars`
/// attaches them to the target's own build command — which is what actually
/// runs cargo, unlike cmake's `set(ENV{...})` (issue 0460).
///
/// Absent pointer (a bare `cargo build` with no lane) → `None`, and the
/// caller's own front-end and default decide, exactly as before. With no board
/// named there IS no platform rung to resolve.
///
/// Every failure here is FATAL rather than a fall-through to defaults: a
/// silently empty tree resolves every knob to a builtin and produces a wrong
/// image with no diagnostic, which is the failure the ladder exists to remove.
/// It lives HERE so the env-pointer dance has ONE spelling — two build scripts
/// resolving the same rungs differently is the drift
/// `check-knob-single-reader` exists to catch, one level up.
/// The platform search path a BUILD SCRIPT should use.
///
/// issue 0979 — this was `default_search_path(&std::env::current_dir(), …)`,
/// and a build script's cwd is its OWN package directory, not the repo root.
/// So `<pkg>/packages/platform` and `<pkg>/config` were the roots searched,
/// neither exists, the tree came back empty with `root: ""`, and the first
/// lookup failed as
///
/// ```text
/// NROS_PLATFORM_NAME=posix: unknown platform `posix`: no /posix/nros-platform.toml
/// ```
///
/// — a platform that does exist, named by the CALLER's request rather than by
/// anything missing, with a leading slash as the only hint that the root was
/// blank. Every Rust fixture in the native lane died this way.
///
/// It is a phase-400 W6 regression with a reason: `nros-node/build.rs` used
/// `nros_build_paths::repo_root()`, and when its copy of the env-pointer dance
/// moved here, the root moved with it to something `nros-board-common` could
/// reach. `current_dir()` was reachable and wrong. `nros-build-paths` is in
/// fact reachable — it is already a dependency of this crate under the same
/// `build-helpers` feature this module lives behind — so the correct resolver
/// comes back, and "where is the repo" keeps its one spelling.
///
/// `try_repo_root` rather than `repo_root`: an out-of-tree consumer has no
/// `nros-sdk-index.toml` to find and must reach the `NROS_PLATFORMS_DIR` arm
/// below instead of panicking on the walk. If neither yields a real directory,
/// [`PlatformsTree::load_search_path`] now says so where the path is resolved.
pub fn build_search_path() -> Vec<PathBuf> {
    let repo = nros_build_paths::try_repo_root().unwrap_or_default();
    PlatformsTree::default_search_path(&repo, std::env::var("NROS_PLATFORMS_DIR").ok().as_deref())
}

pub struct BuildRungs {
    pub platform: String,
    pub tree: PlatformsTree,
    pub board: Option<BoardKnobsFile>,
}

impl BuildRungs {
    /// Resolve from the environment the lane exports, or `None` when no lane
    /// named a platform.
    pub fn from_build_env() -> Option<Self> {
        let platform = std::env::var("NROS_PLATFORM_NAME")
            .ok()
            .filter(|s| !s.is_empty())?;
        println!("cargo:rerun-if-env-changed=NROS_PLATFORM_NAME");

        let board = std::env::var("NROS_BOARD_TOML")
            .ok()
            .filter(|s| !s.is_empty())
            .map(|raw| {
                // issue 0491 — fingerprint the file's CONTENT. A
                // `rerun-if-env-changed` on a variable naming a PATH compares
                // the spelling, and one directory has three spellings here.
                println!("cargo:rerun-if-changed={raw}");
                BoardKnobsFile::load(Path::new(&raw))
                    .unwrap_or_else(|e| panic!("NROS_BOARD_TOML={raw}: {e}"))
            });

        let search = build_search_path();
        let tree = PlatformsTree::load_search_path(&search)
            .unwrap_or_else(|e| panic!("platform search path {search:?}: {e}"));
        Some(Self {
            platform,
            tree,
            board,
        })
    }

    /// A platform with NO `nros-platform.toml` has no rungs, and that is a
    /// normal state — not an error.
    ///
    /// phase-400 W6 REGRESSION, and this is the second half of issue 0979.
    /// `nros-node/build.rs` resolved the platform with `.ok()` before the rungs
    /// moved here; the move turned an absent descriptor into a `panic!`. Three
    /// of the platforms this tree builds — `threadx-linux`, `esp32`,
    /// `zephyr` — have never had a descriptor, so every one of their images
    /// died in a build script:
    ///
    /// ```text
    /// NROS_PLATFORM_NAME=threadx-linux: unknown platform `threadx-linux`:
    ///   no …/packages/platform/threadx-linux/nros-platform.toml
    /// ```
    ///
    /// 0979 fixed the ROOT being empty, which is why the message now names a
    /// real path. It did not fix this: a correct root still has no file for a
    /// platform that declares none.
    ///
    /// What stays fatal is a descriptor that EXISTS and is broken — `Io`,
    /// `Parse`, `Manifest`, a cycle, an arch conflict. Those are a wrong
    /// answer; an absent file is no answer, and no answer means the builtin
    /// defaults every knob already carries. The warning keeps it visible, so a
    /// TYPO in `NROS_PLATFORM_NAME` still shows up rather than silently
    /// selecting builtins — which is the one thing the panic was buying.
    ///
    /// One helper, not three call sites with three spellings: the panic existed
    /// three times over (executor, params, memory) and would have been fixed
    /// once and left twice.
    fn or_builtin_rungs<T: Default>(&self, what: &str, r: Result<T, ConfigError>) -> T {
        match r {
            Ok(v) => v,
            Err(ConfigError::UnknownPlatform { .. }) => {
                println!(
                    "cargo:warning=NROS_PLATFORM_NAME={}: no nros-platform.toml, \
                     so the {what} knobs fall through to their builtin defaults",
                    self.platform
                );
                T::default()
            }
            Err(e) => panic!("NROS_PLATFORM_NAME={}: {e}", self.platform),
        }
    }

    /// The `[knobs.executor]` RUNGS for this build — platform merged with
    /// board, board winning — with NO env rung and no defaults.
    ///
    /// Deliberately not the full ladder: `nros-node/build.rs` composes
    /// env → Kconfig → these → its own builtin, and the Kconfig rung sits
    /// between the front-end and the descriptors. What it needed shared was
    /// the ENV-POINTER DANCE above, not the composition, and pretending
    /// otherwise would have quietly dropped its Kconfig rung.
    pub fn executor_rungs(&self) -> ExecutorKnobs {
        let plat = self.tree.platform_executor_rungs(&self.platform);
        let plat = self.or_builtin_rungs("executor", plat);
        let b = self
            .board
            .as_ref()
            .map(|f| f.knobs.executor.clone())
            .unwrap_or_default();
        ExecutorKnobs {
            max_cbs: b.max_cbs.or(plat.max_cbs),
            max_sc: b.max_sc.or(plat.max_sc),
            max_nodes: b.max_nodes.or(plat.max_nodes),
            max_shutdown_cbs: b.max_shutdown_cbs.or(plat.max_shutdown_cbs),
            action_clients: b.action_clients.or(plat.action_clients),
            arena_size: b.arena_size.or(plat.arena_size),
            subscription_buffer_size: b.subscription_buffer_size.or(plat.subscription_buffer_size),
            param_service_buffer_size: b
                .param_service_buffer_size
                .or(plat.param_service_buffer_size),
        }
    }

    /// The `[knobs.params]` RUNGS for this build — platform merged with board,
    /// board winning, no env rung. Same reason as [`Self::executor_rungs`]:
    /// `nros-params`'s build script composes env → Kconfig → these → its own
    /// builtin, and the Kconfig rung sits between the front-end and the
    /// descriptors.
    pub fn param_rungs(&self) -> ParamKnobs {
        let plat = self.tree.platform_param_rungs(&self.platform);
        let plat = self.or_builtin_rungs("params", plat);
        let b = self
            .board
            .as_ref()
            .map(|f| f.knobs.params.clone())
            .unwrap_or_default();
        ParamKnobs {
            max_parameters: b.max_parameters.or(plat.max_parameters),
            max_param_name_len: b.max_param_name_len.or(plat.max_param_name_len),
            max_string_value_len: b.max_string_value_len.or(plat.max_string_value_len),
            max_array_len: b.max_array_len.or(plat.max_array_len),
            max_byte_array_len: b.max_byte_array_len.or(plat.max_byte_array_len),
        }
    }

    /// The `[knobs.xrce]` RUNGS for this build. See [`Self::rmw_rungs`].
    pub fn xrce_rungs(&self) -> XrceKnobs {
        let plat = self.tree.platform_xrce_rungs(&self.platform);
        let plat = self.or_builtin_rungs("xrce", plat);
        let b = self
            .board
            .as_ref()
            .map(|f| f.knobs.xrce.clone())
            .unwrap_or_default();
        XrceKnobs {
            custom_transport_mtu: b.custom_transport_mtu.or(plat.custom_transport_mtu),
            stream_history: b.stream_history.or(plat.stream_history),
        }
    }

    /// The `[knobs.zenoh.limits]` RUNGS for this build. See [`Self::rmw_rungs`].
    pub fn zenoh_limit_rungs(&self) -> ZenohLimitKnobs {
        let plat = self.tree.platform_zenoh_limit_rungs(&self.platform);
        let plat = self.or_builtin_rungs("zenoh.limits", plat);
        let b = self
            .board
            .as_ref()
            .map(|f| f.knobs.zenoh.limits.clone())
            .unwrap_or_default();
        ZenohLimitKnobs {
            keyexpr_string_size: b.keyexpr_string_size.or(plat.keyexpr_string_size),
            service_timeout_ms: b.service_timeout_ms.or(plat.service_timeout_ms),
            subscriber_ring_depth: b.subscriber_ring_depth.or(plat.subscriber_ring_depth),
        }
    }

    /// The `[knobs.zenoh.wire]` RUNGS for this build — platform merged with
    /// board, board winning. See [`Self::rmw_rungs`].
    pub fn wire_rungs(&self) -> WireKnobs {
        let plat = self.tree.platform_wire_rungs(&self.platform);
        let plat = self.or_builtin_rungs("zenoh.wire", plat);
        let b = self
            .board
            .as_ref()
            .map(|f| f.knobs.zenoh.wire.clone())
            .unwrap_or_default();
        WireKnobs {
            batch_unicast_size: b.batch_unicast_size.or(plat.batch_unicast_size),
            batch_multicast_size: b.batch_multicast_size.or(plat.batch_multicast_size),
            frag_max_size: b.frag_max_size.or(plat.frag_max_size),
            get_reply_buf_size: b.get_reply_buf_size.or(plat.get_reply_buf_size),
            get_poll_interval_ms: b.get_poll_interval_ms.or(plat.get_poll_interval_ms),
        }
    }

    /// The `[knobs.runtime]` RUNGS for this build — platform merged with board,
    /// board winning. See [`Self::rmw_rungs`].
    pub fn runtime_rungs(&self) -> RuntimeKnobs {
        let plat = self.tree.platform_runtime_rungs(&self.platform);
        let plat = self.or_builtin_rungs("runtime", plat);
        let b = self
            .board
            .as_ref()
            .map(|f| f.knobs.runtime.clone())
            .unwrap_or_default();
        RuntimeKnobs {
            max_components: b.max_components.or(plat.max_components),
            component_slot_bytes: b.component_slot_bytes.or(plat.component_slot_bytes),
            max_class_instances: b.max_class_instances.or(plat.max_class_instances),
            max_cell_entities: b.max_cell_entities.or(plat.max_cell_entities),
            let_buffer_size: b.let_buffer_size.or(plat.let_buffer_size),
        }
    }

    /// The `[knobs.net]` RUNGS for this build — platform merged with board,
    /// board winning. See [`Self::rmw_rungs`].
    pub fn net_rungs(&self) -> NetKnobs {
        let plat = self.tree.platform_net_rungs(&self.platform);
        let plat = self.or_builtin_rungs("net", plat);
        let b = self
            .board
            .as_ref()
            .map(|f| f.knobs.net.clone())
            .unwrap_or_default();
        NetKnobs {
            max_sockets: b.max_sockets.or(plat.max_sockets),
            max_udp_sockets: b.max_udp_sockets.or(plat.max_udp_sockets),
            buffer_size: b.buffer_size.or(plat.buffer_size),
            connect_timeout_ms: b.connect_timeout_ms.or(plat.connect_timeout_ms),
            socket_timeout_ms: b.socket_timeout_ms.or(plat.socket_timeout_ms),
        }
    }

    /// The `[knobs.rmw]` RUNGS for this build — platform merged with board,
    /// board winning — with no env rung and no defaults.
    ///
    /// Same shape as [`Self::param_rungs`]: the consuming build script composes
    /// env -> Kconfig -> these -> its own builtin, and keeps its RANGE checks,
    /// which are a property of the array it carves and not of the ladder.
    pub fn rmw_rungs(&self) -> RmwKnobs {
        let plat = self.tree.platform_rmw_rungs(&self.platform);
        let plat = self.or_builtin_rungs("rmw", plat);
        let b = self
            .board
            .as_ref()
            .map(|f| f.knobs.rmw.clone())
            .unwrap_or_default();
        RmwKnobs {
            max_backends: b.max_backends.or(plat.max_backends),
            max_nodes: b.max_nodes.or(plat.max_nodes),
            message_info_slots: b.message_info_slots.or(plat.message_info_slots),
        }
    }

    /// The memory tenant for this build, over the full ladder.
    pub fn memory(&self, defaults: &[(&'static str, usize)]) -> Vec<(&'static str, ResolvedUsize)> {
        let plat = self.tree.platform_memory_rungs(&self.platform);
        let plat = self.or_builtin_rungs("memory", plat);
        self.tree.resolve_memory_from(
            &self.platform,
            &plat,
            self.board.as_ref().map(|b| &b.knobs.memory),
            &|k| std::env::var(k).ok(),
            defaults,
        )
    }

    /// One resolved memory knob, by name.
    pub fn memory_value(&self, knob: &'static str, default: usize) -> usize {
        self.memory(&[(knob, default)])
            .into_iter()
            .next()
            .map(|(_, r)| r.value)
            .unwrap_or(default)
    }
}

/// The env front-end name for an executor knob. These are the EXISTING names
/// `nros-node/build.rs` already reads, kept verbatim: migrating a knob into the
/// ladder must not change how anyone sets it.
pub fn executor_env_key(knob: &str) -> &'static str {
    match knob {
        "max_cbs" => "NROS_EXECUTOR_MAX_CBS",
        "max_sc" => "NROS_EXECUTOR_MAX_SC",
        "max_nodes" => "NROS_EXECUTOR_MAX_NODES",
        "max_shutdown_cbs" => "NROS_EXECUTOR_MAX_SHUTDOWN_CBS",
        "action_clients" => "NROS_EXECUTOR_ACTION_CLIENTS",
        "arena_size" => "NROS_EXECUTOR_ARENA_SIZE",
        "subscription_buffer_size" => "NROS_SUBSCRIPTION_BUFFER_SIZE",
        "param_service_buffer_size" => "NROS_PARAM_SERVICE_BUFFER_SIZE",
        other => panic!("unknown executor knob `{other}`"),
    }
}

/// phase-400 W8 — the env front-end for one executor knob, with NO ladder.
///
/// For callers that cannot reach a platform tree (an out-of-tree consumer, or a
/// check with no platform named) but must still honour an operator's override.
/// It lives HERE, beside `executor_env_key`, so the env-reading idiom for a
/// migrated knob exists in exactly one crate — which is what
/// `check-knob-single-reader` enforces, and why a caller that inlined
/// `std::env::var("NROS_EXECUTOR_MAX_CBS")` is a finding rather than a style
/// nit: two readers can disagree and nothing reports it.
pub fn executor_env_only(knob: &str, default: usize) -> usize {
    std::env::var(executor_env_key(knob))
        .ok()
        .and_then(|v| v.trim().parse::<usize>().ok())
        .unwrap_or(default)
}

/// `[knobs.params]` — phase-400 W6, the parameter-storage tenant.
///
/// Five bounds on what a parameter server can hold. They vary by BOARD in
/// practice and the tree records a case: phase-292's ASI consumer needed
/// `NROS_MAX_PARAMETERS=256` and set it in a `build.sh`, which is a board fact
/// living in a shell script because there was nowhere to declare it.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ParamKnobs {
    #[serde(default)]
    pub max_parameters: Option<usize>,
    #[serde(default)]
    pub max_param_name_len: Option<usize>,
    #[serde(default)]
    pub max_string_value_len: Option<usize>,
    #[serde(default)]
    pub max_array_len: Option<usize>,
    #[serde(default)]
    pub max_byte_array_len: Option<usize>,
}

/// phase-400 W6 — the RMW static-pool tenant.
///
/// Three fixed-size arrays the CFFI registry and node table are carved from,
/// read by `packages/rmw/cffi/build.rs`. They are a PLATFORM fact: an embedded
/// image links one backend and runs a handful of nodes, a host build links
/// several and may run many, and until now the only way to say so was an env
/// var on the shell that happened to run the build.
///
/// `NROS_RMW_SUBSCRIBER_SLOTS` is deliberately NOT here. It sits in the same
/// build script and looks identical, but phase-412 W1 derives it from the
/// entity inventory (`COUNT_SUBSCRIPTION`), and a knob two campaigns resolve is
/// the drift issue 0938 cost. Checked before writing this rather than after.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RmwKnobs {
    #[serde(default)]
    pub max_backends: Option<usize>,
    #[serde(default)]
    pub max_nodes: Option<usize>,
    #[serde(default)]
    pub message_info_slots: Option<usize>,
}

/// phase-400 W6 — the smoltcp (bare-metal net) tenant.
///
/// Socket pools, the per-socket buffer, and the two timeouts, read by
/// `packages/drivers/net/nros-smoltcp/build.rs`. Every one is a PLATFORM fact:
/// a brokered bare-metal client needs one TCP and one UDP socket, an RTPS one
/// needs four, and until now the only way to say so was an env var on the shell
/// that happened to run the build.
///
/// THIS TENANT IS WHY THE READER IS A LEAF CRATE. `nros-smoltcp` could not
/// depend on `nros-board-common` — cargo counts an optional dependency when it
/// looks for cycles, and the board crate reaches back through `nros-platform ->
/// nros-platform-esp32-qemu -> nros-smoltcp`. Every driver was locked out of
/// the ladder until the reader moved here.
///
/// Note what the rung does NOT replace: the crate's default for
/// `max_udp_sockets` is FEATURE-derived (1 brokered, 4 with `rtps`), and that
/// stays the builtin. A descriptor naming the knob outranks it — an explicit
/// declaration beats a heuristic — but a board that says nothing still gets the
/// feature-appropriate number rather than a constant.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NetKnobs {
    #[serde(default)]
    pub max_sockets: Option<usize>,
    #[serde(default)]
    pub max_udp_sockets: Option<usize>,
    #[serde(default)]
    pub buffer_size: Option<usize>,
    #[serde(default)]
    pub connect_timeout_ms: Option<usize>,
    #[serde(default)]
    pub socket_timeout_ms: Option<usize>,
}

/// phase-400 W6 — the component-runtime tenant.
///
/// The four static pools `packages/api/nros/build.rs` carves the component
/// runtime from: how many components an image may register, how big each one's
/// slot is, how many instances of a class, and how many entities per cell.
///
/// phase-391 CONSUMES these (it emits `config::MAX_COMPONENTS` and friends and
/// sizes the arena from them); it does not own their VALUES, which is the same
/// relationship `nros-node/build.rs` had with the executor knobs before this
/// wave. So the rungs are the ladder's to give and the consts stay phase-391's
/// to emit.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RuntimeKnobs {
    #[serde(default)]
    pub max_components: Option<usize>,
    #[serde(default)]
    pub component_slot_bytes: Option<usize>,
    #[serde(default)]
    pub max_class_instances: Option<usize>,
    #[serde(default)]
    pub max_cell_entities: Option<usize>,
    /// The logical-execution-time staging buffer, read by
    /// `nros-build-helpers`'s C emitter. Grouped here because LET is an
    /// execution-model buffer, not a wire or transport size.
    #[serde(default)]
    pub let_buffer_size: Option<usize>,
}

/// Every runtime knob, in a stable order. Same reason as [`EXECUTOR_KNOBS`].
pub const RUNTIME_KNOBS: &[&str] = &[
    "max_components",
    "component_slot_bytes",
    "max_class_instances",
    "max_cell_entities",
    "let_buffer_size",
];

/// The env front-end for a runtime knob — the EXISTING names, verbatim.
pub fn runtime_env_key(knob: &str) -> &'static str {
    match knob {
        "max_components" => "NROS_RUNTIME_MAX_COMPONENTS",
        "component_slot_bytes" => "NROS_RUNTIME_COMPONENT_SLOT_BYTES",
        "max_class_instances" => "NROS_RUNTIME_MAX_CLASS_INSTANCES",
        "max_cell_entities" => "NROS_RUNTIME_MAX_CELL_ENTITIES",
        "let_buffer_size" => "NROS_LET_BUFFER_SIZE",
        other => panic!("unknown runtime knob `{other}`"),
    }
}

/// Every net knob, in a stable order. Same reason as [`EXECUTOR_KNOBS`].
pub const NET_KNOBS: &[&str] = &[
    "max_sockets",
    "max_udp_sockets",
    "buffer_size",
    "connect_timeout_ms",
    "socket_timeout_ms",
];

/// The env front-end for a net knob — the EXISTING `NROS_SMOLTCP_*` names.
///
/// The `ZPICO_SMOLTCP_*` spellings the build script also accepts are a
/// DEPRECATED alias, not a second front-end: they rank WITH env, above this
/// tenant's rungs, and are not listed here because nothing should start using
/// them.
pub fn net_env_key(knob: &str) -> &'static str {
    match knob {
        "max_sockets" => "NROS_SMOLTCP_MAX_SOCKETS",
        "max_udp_sockets" => "NROS_SMOLTCP_MAX_UDP_SOCKETS",
        "buffer_size" => "NROS_SMOLTCP_BUFFER_SIZE",
        "connect_timeout_ms" => "NROS_SMOLTCP_CONNECT_TIMEOUT_MS",
        "socket_timeout_ms" => "NROS_SMOLTCP_SOCKET_TIMEOUT_MS",
        other => panic!("unknown net knob `{other}`"),
    }
}

/// Every RMW knob, in a stable order. Same reason as [`EXECUTOR_KNOBS`].
pub const RMW_KNOBS: &[&str] = &["max_backends", "max_nodes", "message_info_slots"];

/// The env front-end for an RMW knob — the EXISTING names, verbatim.
pub fn rmw_env_key(knob: &str) -> &'static str {
    match knob {
        "max_backends" => "NROS_RMW_MAX_BACKENDS",
        "max_nodes" => "NROS_RMW_MAX_NODES",
        "message_info_slots" => "NROS_RMW_MESSAGE_INFO_SLOTS",
        other => panic!("unknown rmw knob `{other}`"),
    }
}

/// Every parameter knob, in a stable order. Same reason as [`EXECUTOR_KNOBS`].
pub const PARAM_KNOBS: &[&str] = &[
    "max_parameters",
    "max_param_name_len",
    "max_string_value_len",
    "max_array_len",
    "max_byte_array_len",
];

/// The env front-end for a parameter knob — the EXISTING names, verbatim.
pub fn param_env_key(knob: &str) -> &'static str {
    match knob {
        "max_parameters" => "NROS_MAX_PARAMETERS",
        "max_param_name_len" => "NROS_MAX_PARAM_NAME_LEN",
        "max_string_value_len" => "NROS_MAX_STRING_VALUE_LEN",
        "max_array_len" => "NROS_MAX_ARRAY_LEN",
        "max_byte_array_len" => "NROS_MAX_BYTE_ARRAY_LEN",
        other => panic!("unknown param knob `{other}`"),
    }
}

/// The zenoh tx tenant with NO platform rung — the env front-end over the
/// builtins, and honest about which one answered.
///
/// phase-400 W8. `nros-zpico-build` had its own copy of this for the case where
/// no platform name resolves, which made three migrated knobs have two readers
/// that could disagree — the shape `check-knob-single-reader` exists to catch,
/// and the shape issues 0135 and 0316 both were.
///
/// That copy also reported `KnobSource::Env` for a value that came from a
/// BUILTIN, so `nros config explain` would have named the wrong rung. Here the
/// source is whichever actually answered.
///
/// Same role as [`executor_env_only`], and the same reasoning: a caller that
/// cannot reach a platform tree still honours an operator's override, through
/// one implementation rather than its own.
#[must_use]
pub fn tx_env_only(env: &dyn Fn(&str) -> Option<String>) -> ResolvedTxKnobs {
    let flag =
        |name: &str, builtin: bool| match env(name).and_then(|v| v.trim().parse::<u64>().ok()) {
            Some(n) => Resolved {
                value: n != 0,
                source: KnobSource::Env,
            },
            None => Resolved {
                value: builtin,
                source: KnobSource::Builtin,
            },
        };
    ResolvedTxKnobs {
        batch: flag("ZPICO_TX_BATCH", BUILTIN_TX_BATCH),
        split_lock: flag("ZPICO_TX_SPLIT_LOCK", BUILTIN_TX_SPLIT_LOCK),
        flush_ms: match env("ZPICO_TX_BATCH_FLUSH_MS").and_then(|v| v.trim().parse::<u64>().ok()) {
            Some(n) => Resolved {
                value: n,
                source: KnobSource::Env,
            },
            None => Resolved {
                value: BUILTIN_TX_FLUSH_MS,
                source: KnobSource::Builtin,
            },
        },
    }
}

/// `[knobs.executor]` — phase-400 W6. All optional; `None` defers to the rung
/// below, and the built-in defaults stay in `nros-node/build.rs` so an absent
/// tree changes nothing.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExecutorKnobs {
    pub max_cbs: Option<usize>,
    pub max_sc: Option<usize>,
    pub max_nodes: Option<usize>,
    pub max_shutdown_cbs: Option<usize>,
    pub action_clients: Option<usize>,
    pub arena_size: Option<usize>,
    pub subscription_buffer_size: Option<usize>,
    pub param_service_buffer_size: Option<usize>,
}

/// `[knobs.memory]` — phase-400 W6, the platform memory tenant.
///
/// The RTOS heap and the application task's stack: two numbers that genuinely
/// vary by PLATFORM and BOARD and that no derivation campaign owns, which is
/// what makes them the ladder's business rather than phase-392's or
/// phase-403's.
///
/// **Stored in BYTES, always.** Their env front-ends disagree about units —
/// `NROS_FREERTOS_HEAP_KB` and `NROS_FREERTOS_APP_STACK_KB` are KiB,
/// `NROS_ZEPHYR_HEAP_SIZE` is bytes — and a table where "heap" means one thing
/// on one platform and another elsewhere is a unit bug waiting to be written.
/// The front-ends keep their spellings and convert; the ladder has one unit.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MemoryKnobs {
    /// The RTOS heap this platform sizes, in bytes.
    #[serde(default)]
    pub heap_bytes: Option<usize>,
    /// The application task's stack, in bytes.
    #[serde(default)]
    pub app_stack_bytes: Option<usize>,
}

/// Every memory knob, in a stable order. Same reason as [`EXECUTOR_KNOBS`].
pub const MEMORY_KNOBS: &[&str] = &["heap_bytes", "app_stack_bytes"];

/// The env front-end for a memory knob. As with the executor tenant these are
/// the EXISTING names, kept verbatim — migrating a knob into the ladder must
/// not change how anyone sets it.
///
/// The FreeRTOS pair is the platform that sizes both today; Zephyr's heap has
/// its own spelling and is resolved by `nros-platform`'s build script.
pub fn memory_env_key(knob: &str) -> &'static str {
    memory_env_key_for("freertos", knob)
}

/// The env front-end for a memory knob ON A GIVEN PLATFORM.
///
/// The same knob has different front-end spellings per platform — a heap is
/// `NROS_FREERTOS_HEAP_KB` on FreeRTOS and `NROS_ZEPHYR_HEAP_SIZE` on Zephyr —
/// because those names predate the ladder and migrating a knob must not change
/// how anyone sets it. The RUNG is one knob; only its front-end differs, so
/// this is where the difference lives rather than in two knob names.
///
/// Without this, `nros config explain --platform zephyr` would print the
/// FreeRTOS variable beside the Zephyr heap — telling a reader to set something
/// that does nothing, in the report whose whole job is to say where a value
/// came from.
pub fn memory_env_key_for(platform: &str, knob: &str) -> &'static str {
    match (platform, knob) {
        ("zephyr", "heap_bytes") => "NROS_ZEPHYR_HEAP_SIZE",
        (_, "heap_bytes") => "NROS_FREERTOS_HEAP_KB",
        (_, "app_stack_bytes") => "NROS_FREERTOS_APP_STACK_KB",
        (_, other) => panic!("unknown memory knob `{other}`"),
    }
}

/// Whether a memory knob's env front-end is spelled in KiB rather than bytes.
/// The ladder stores bytes; this is the one place the difference lives.
pub fn memory_env_is_kib(knob: &str) -> bool {
    memory_env_is_kib_for("freertos", knob)
}

/// Whether a memory knob's front-end is KiB on this platform. The unit travels
/// with the SPELLING, not with the knob: `NROS_FREERTOS_HEAP_KB` is KiB and
/// `NROS_ZEPHYR_HEAP_SIZE` is bytes, for the same `heap_bytes` rung.
pub fn memory_env_is_kib_for(platform: &str, knob: &str) -> bool {
    !matches!((platform, knob), ("zephyr", "heap_bytes"))
        && matches!(knob, "heap_bytes" | "app_stack_bytes")
}

/// One resolved executor knob: value, rung, and the env name that is still its
/// front-end. The name travels with the value so `explain` can tell a reader
/// which variable to set, without a second table to keep in step.
#[derive(Debug, Clone)]
pub struct ResolvedUsize {
    pub value: usize,
    pub source: KnobSource,
    pub env_key: &'static str,
}

/// `[knobs.transport]` — RFC-0086 D1.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TransportKnobs {
    /// `exactly-one-of` serial | tcp | udp. `None` defers to the rung below.
    pub kind: Option<String>,
    /// Opaque to this layer: the BACKEND lowers it into its own locator
    /// spelling, and the BOARD resolves a peripheral name. Carried, not parsed.
    pub endpoint: Option<String>,
}

/// The transports the resolver knows how to constrain. RFC-0086 D2 makes this
/// an `exactly-one-of` group, which is why an unknown value is an error rather
/// than a pass-through: a typo that silently selects nothing would leave every
/// implication unapplied and the image would build with the wrong links on.
pub const TRANSPORT_KINDS: &[&str] = &["serial", "tcp", "udp"];

/// Built-in default — level 1. `tcp` preserves the behaviour of every image
/// that predates this tenant, where `NROS_ZENOH_LINK_TCP` was `default y`.
pub const BUILTIN_TRANSPORT_KIND: &str = "tcp";

/// One knob set by an implication rather than by a ladder rung.
///
/// RFC-0086 D2: `implies` is Kconfig `imply` strength, NEVER `select` — a
/// higher rung still wins, and when it does the override is recorded here so
/// `nros config explain` can report it. A forcing verb would let
/// `transport.kind = "serial"` silently stamp out an explicitly requested TCP
/// link, which is the failure mode Kconfig's own documentation records for
/// `select`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Implied {
    /// Dotted knob name, e.g. `links.tcp` or `drivers.ethernet`.
    pub knob: String,
    /// The value the implication asks for.
    pub value: bool,
    /// The rule that asked, for the explain output.
    pub rule: String,
    /// Set when a higher rung contradicted the implication. The implication
    /// LOSES; this records that it happened.
    pub overridden_by: Option<KnobSource>,
}

/// The fully-resolved transport tenant.
#[derive(Debug, Clone)]
pub struct ResolvedTransport {
    pub kind: Resolved<String>,
    pub endpoint: Resolved<Option<String>>,
    /// Every knob this transport choice implies, in declaration order.
    pub implied: Vec<Implied>,
}

#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ZenohKnobs {
    #[serde(default)]
    pub tx: TxKnobs,
    /// phase-400 W6 — the WIRE sizes: batch buffers, the fragmentation
    /// ceiling, the get-reply staging block and its poll interval.
    ///
    /// Deliberately NOT the entity caps. `ZPICO_MAX_QUERYABLES`,
    /// `ZPICO_MAX_SESSIONS` and `SERVICE_BUFFERS` are phase-392's question —
    /// that phase is deciding whether they stop being globals at all, and a
    /// per-platform rung would answer it the wrong way. These five are sizes of
    /// the wire itself, which no campaign derives.
    #[serde(default)]
    pub wire: WireKnobs,
    /// phase-400 W6 — zenoh runtime limits. See [`ZenohLimitKnobs`].
    #[serde(default)]
    pub limits: ZenohLimitKnobs,
}

/// phase-400 W6 — the zenoh WIRE sizes, read by `nros-zpico-build`.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WireKnobs {
    #[serde(default)]
    pub batch_unicast_size: Option<usize>,
    #[serde(default)]
    pub batch_multicast_size: Option<usize>,
    #[serde(default)]
    pub frag_max_size: Option<usize>,
    #[serde(default)]
    pub get_reply_buf_size: Option<usize>,
    #[serde(default)]
    pub get_poll_interval_ms: Option<usize>,
}

/// phase-400 W6 — the XRCE transport tenant, read by `nros-rmw-xrce-cffi`.
///
/// The MTU is a TRANSPORT fact (a serial link's frame budget differs from
/// UDP's) and the stream history is the reliable-stream depth the agent
/// negotiates against. Both are per-platform in practice and were env-only.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct XrceKnobs {
    #[serde(default)]
    pub custom_transport_mtu: Option<usize>,
    #[serde(default)]
    pub stream_history: Option<usize>,
}

/// Every XRCE knob, in a stable order. Same reason as [`EXECUTOR_KNOBS`].
pub const XRCE_KNOBS: &[&str] = &["custom_transport_mtu", "stream_history"];

/// The env front-end for an XRCE knob — the EXISTING names, verbatim.
pub fn xrce_env_key(knob: &str) -> &'static str {
    match knob {
        "custom_transport_mtu" => "NROS_XRCE_CUSTOM_TRANSPORT_MTU",
        "stream_history" => "NROS_XRCE_STREAM_HISTORY",
        other => panic!("unknown xrce knob `{other}`"),
    }
}

/// phase-400 W6 — zenoh RUNTIME limits, read by `nros-rmw-zenoh`.
///
/// Separate from [`WireKnobs`] on purpose: those size the WIRE (batching,
/// fragmentation), these bound what a session holds — the key-expression
/// string, the default RPC timeout, and the per-subscriber SPSC ring.
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ZenohLimitKnobs {
    #[serde(default)]
    pub keyexpr_string_size: Option<usize>,
    #[serde(default)]
    pub service_timeout_ms: Option<usize>,
    #[serde(default)]
    pub subscriber_ring_depth: Option<usize>,
}

/// Every zenoh limit knob, in a stable order.
pub const ZENOH_LIMIT_KNOBS: &[&str] = &[
    "keyexpr_string_size",
    "service_timeout_ms",
    "subscriber_ring_depth",
];

/// The env front-end for a zenoh limit knob — the EXISTING names, verbatim.
pub fn zenoh_limit_env_key(knob: &str) -> &'static str {
    match knob {
        "keyexpr_string_size" => "NROS_KEYEXPR_STRING_SIZE",
        "service_timeout_ms" => "NROS_SERVICE_TIMEOUT_MS",
        "subscriber_ring_depth" => "ZPICO_SUBSCRIBER_RING_DEPTH",
        other => panic!("unknown zenoh limit knob `{other}`"),
    }
}

/// Every wire knob, in a stable order. Same reason as [`EXECUTOR_KNOBS`].
pub const WIRE_KNOBS: &[&str] = &[
    "batch_unicast_size",
    "batch_multicast_size",
    "frag_max_size",
    "get_reply_buf_size",
    "get_poll_interval_ms",
];

/// The env front-end for a wire knob — the EXISTING `ZPICO_*` names.
pub fn wire_env_key(knob: &str) -> &'static str {
    match knob {
        "batch_unicast_size" => "ZPICO_BATCH_UNICAST_SIZE",
        "batch_multicast_size" => "ZPICO_BATCH_MULTICAST_SIZE",
        "frag_max_size" => "ZPICO_FRAG_MAX_SIZE",
        "get_reply_buf_size" => "ZPICO_GET_REPLY_BUF_SIZE",
        "get_poll_interval_ms" => "ZPICO_GET_POLL_INTERVAL_MS",
        other => panic!("unknown wire knob `{other}`"),
    }
}

/// The phase-282 TX levers. All optional — `None` means "defer to the
/// rung below".
#[derive(Debug, Default, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TxKnobs {
    pub batch: Option<bool>,
    pub split_lock: Option<bool>,
    pub flush_ms: Option<u64>,
}

/// Which ladder rung produced a resolved value (for
/// `nros config explain` + capability-check diagnostics).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KnobSource {
    Builtin,
    Platform,
    Board,
    Env,
}

impl KnobSource {
    pub fn as_str(self) -> &'static str {
        match self {
            KnobSource::Builtin => "builtin",
            KnobSource::Platform => "platform",
            KnobSource::Board => "board",
            KnobSource::Env => "env",
        }
    }
}

/// A resolved knob value + the rung that set it.
#[derive(Debug, Clone, Copy)]
pub struct Resolved<T> {
    pub value: T,
    pub source: KnobSource,
}

/// The fully-resolved `zenoh.tx` knob set.
#[derive(Debug, Clone)]
pub struct ResolvedTxKnobs {
    pub batch: Resolved<bool>,
    pub split_lock: Resolved<bool>,
    pub flush_ms: Resolved<u64>,
}

/// Built-in defaults — level 1 of the ladder. MUST equal the historical
/// hardcoded env defaults so an empty tree changes nothing (W2.c).
pub const BUILTIN_TX_BATCH: bool = false;
pub const BUILTIN_TX_SPLIT_LOCK: bool = false;
pub const BUILTIN_TX_FLUSH_MS: u64 = 50;

/// A loaded tree of platform config files (`<root>/<name>/nros-platform.toml`).
#[derive(Debug, Default)]
pub struct PlatformsTree {
    files: BTreeMap<String, PlatformConfigFile>,
    /// Merged `[arch.*]` table across all files (identical duplicates
    /// tolerated).
    arch: BTreeMap<String, ArchEntry>,
    root: PathBuf,
}

/// Load / resolution errors. `Manifest` wraps the underlying shared
/// parser's error type for the `[build.zenoh]` payload.
#[derive(Debug)]
pub enum ConfigError {
    /// phase-400 W3 — `transport.kind` outside the `exactly-one-of` group.
    UnknownTransport {
        platform: String,
        kind: String,
        known: String,
    },
    /// phase-400 W3 — `requires` failed: the transport needs a capability the
    /// platform does not declare. Named at CONFIG time, which is the whole
    /// point: the alternative is a link error against `AF_INET` much later.
    TransportCapabilityMissing {
        platform: String,
        kind: String,
        capability: String,
        source: String,
    },
    Io {
        path: String,
        source: std::io::Error,
    },
    Parse {
        path: String,
        source: toml::de::Error,
    },
    Manifest(ManifestError),
    /// issue 0979 — every root in the search path was missing, so the tree
    /// would be EMPTY and no platform could ever resolve from it. Reported
    /// where the path is resolved rather than several frames later as
    /// `UnknownPlatform` for a platform that does exist.
    NoSearchRoot {
        tried: String,
    },
    UnknownPlatform {
        name: String,
        root: String,
    },
    InheritsCycle {
        name: String,
    },
    ArchConflict {
        name: String,
        file_a: String,
        file_b: String,
    },
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::UnknownTransport {
                platform,
                kind,
                known,
            } => write!(
                f,
                "platform `{platform}`: knobs.transport.kind = `{kind}` is not one of: {known}"
            ),
            ConfigError::TransportCapabilityMissing {
                platform,
                kind,
                capability,
                source,
            } => write!(
                f,
                "platform `{platform}`: transport.kind = `{kind}` (from {source}) requires \
                 capabilities.{capability}, which this platform does not declare. Either pick a \
                 transport the platform supports, or add the capability to its nros-platform.toml \
                 if the fact is simply missing."
            ),
            ConfigError::Io { path, source } => write!(f, "{path}: {source}"),
            ConfigError::Parse { path, source } => write!(f, "{path}: {source}"),
            ConfigError::Manifest(e) => write!(f, "{e}"),
            ConfigError::NoSearchRoot { tried } => write!(
                f,
                "no platform descriptor root exists; tried {tried}. \
                 Every root in the search path is missing, so the tree is empty \
                 and no platform can resolve from it — this is a wrong ROOT, not \
                 a missing platform (issue 0979)."
            ),
            ConfigError::UnknownPlatform { name, root } => write!(
                f,
                "unknown platform `{name}`: no {root}/{name}/{PLATFORM_CONFIG_FILENAME}"
            ),
            ConfigError::InheritsCycle { name } => {
                write!(f, "platform `{name}`: `inherits` cycle")
            }
            ConfigError::ArchConflict {
                name,
                file_a,
                file_b,
            } => write!(
                f,
                "arch profile `{name}` defined differently in {file_a} and {file_b} \
                 — profiles duplicated across platform files must be identical"
            ),
        }
    }
}

impl std::error::Error for ConfigError {}

impl From<ManifestError> for ConfigError {
    fn from(e: ManifestError) -> Self {
        ConfigError::Manifest(e)
    }
}

impl PlatformsTree {
    /// Load every `<root>/*/nros-platform.toml`. Directories without the
    /// file are skipped (a platform package may predate its config file).
    /// phase-400 W1 / RFC-0086 D3 — load platform descriptors from a SEARCH
    /// PATH rather than one directory.
    ///
    /// RFC-0049 opens by rejecting a central file: an out-of-tree platform
    /// *"cannot join `zenoh_platforms.toml` without forking the tree."* The rmw
    /// and board axes honour that; the platform axis did not, because `load`
    /// takes exactly one root. A third-party platform therefore had to fork
    /// `config/` or repoint the whole root and lose the in-tree platforms with
    /// it.
    ///
    /// Roots are searched in order and the FIRST definition of a name wins, so
    /// a caller can shadow an in-tree platform by putting its own root first.
    /// A later root re-defining a name is skipped silently rather than
    /// erroring: that is what "shadow" means, and it is the same precedence
    /// rule RFC-0071 D5 uses for backends.
    ///
    /// Missing roots are skipped, not fatal — a search path naturally contains
    /// entries that do not exist on every machine.
    pub fn load_search_path(roots: &[PathBuf]) -> Result<Self, ConfigError> {
        let mut merged: Option<PlatformsTree> = None;
        for root in roots {
            if !root.is_dir() {
                continue;
            }
            let tree = Self::load(root)?;
            match merged.as_mut() {
                None => merged = Some(tree),
                Some(acc) => {
                    for (name, file) in tree.files {
                        acc.files.entry(name).or_insert(file);
                    }
                    for (k, v) in tree.arch {
                        acc.arch.entry(k).or_insert(v);
                    }
                }
            }
        }
        // issue 0979 — an all-missing search path is an ERROR here, not an
        // empty tree returned to a caller that will look something up in it.
        //
        // "Missing roots are skipped" is right for a search PATH: a porter
        // prepends their own tree and the in-tree ones may or may not be
        // there. It is not right for ALL of them being absent, which means the
        // ROOT was computed wrong. The empty tree that used to be returned
        // carries `root: ""`, so the failure surfaced later as
        // `unknown platform \`posix\`: no /posix/nros-platform.toml` — a
        // message naming a platform that exists, with a leading slash as the
        // only clue that the root was blank.
        merged.ok_or_else(|| ConfigError::NoSearchRoot {
            tried: if roots.is_empty() {
                "(an empty search path)".to_string()
            } else {
                roots
                    .iter()
                    .map(|p| p.display().to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            },
        })
    }

    /// The default search path: `$NROS_PLATFORM_PATH` entries first (colon
    /// separated, so a porter can prepend their own tree), then the two in-tree
    /// homes. `packages/platform` comes before `config` so a descriptor that
    /// has moved beside its crate wins over a stale copy left behind.
    pub fn default_search_path(repo_root: &Path, env_path: Option<&str>) -> Vec<PathBuf> {
        let mut out: Vec<PathBuf> = Vec::new();
        if let Some(p) = env_path {
            out.extend(p.split(':').filter(|s| !s.is_empty()).map(PathBuf::from));
        }
        out.push(repo_root.join("packages/platform"));
        out.push(repo_root.join("config"));
        out
    }

    pub fn load(root: &Path) -> Result<Self, ConfigError> {
        let mut tree = PlatformsTree {
            root: root.to_path_buf(),
            ..Default::default()
        };
        let entries = fs::read_dir(root).map_err(|e| ConfigError::Io {
            path: root.display().to_string(),
            source: e,
        })?;
        let mut arch_origin: BTreeMap<String, String> = BTreeMap::new();
        let mut dirs: Vec<PathBuf> = entries
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.is_dir())
            .collect();
        dirs.sort();
        for dir in dirs {
            let file = dir.join(PLATFORM_CONFIG_FILENAME);
            if !file.is_file() {
                continue;
            }
            let name = dir
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or_default()
                .to_string();
            let text = fs::read_to_string(&file).map_err(|e| ConfigError::Io {
                path: file.display().to_string(),
                source: e,
            })?;
            let parsed: PlatformConfigFile =
                toml::from_str(&text).map_err(|e| ConfigError::Parse {
                    path: file.display().to_string(),
                    source: e,
                })?;
            for (arch_name, entry) in &parsed.arch {
                match tree.arch.get(arch_name) {
                    None => {
                        tree.arch.insert(arch_name.clone(), entry.clone());
                        arch_origin.insert(arch_name.clone(), name.clone());
                    }
                    Some(existing) => {
                        // Identical duplicates tolerated (shared profiles
                        // like cortex-m3); conflicting redefinition is a
                        // drift bug.
                        if format!("{existing:?}") != format!("{entry:?}") {
                            return Err(ConfigError::ArchConflict {
                                name: arch_name.clone(),
                                file_a: arch_origin.get(arch_name).cloned().unwrap_or_default(),
                                file_b: name.clone(),
                            });
                        }
                    }
                }
            }
            // phase-400 W1 — key by the descriptor's own canonical name when it
            // declares one, falling back to the directory. A platform that has
            // moved beside its crate lives in `nros-platform-<x>/`, so keying
            // on the directory alone would make it resolve as
            // `nros-platform-zephyr` and no longer answer to `zephyr`. The
            // `names` field already exists for exactly this (phase-349 W1 /
            // RFC-0072); this makes the loader honour it.
            let key = parsed
                .names
                .first()
                .cloned()
                .unwrap_or_else(|| name.clone());
            tree.files.insert(key, parsed);
        }
        Ok(tree)
    }

    /// Platform names present in the tree.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.files.keys().map(String::as_str)
    }

    /// The merged `[arch.*]` table.
    pub fn arch_table(&self) -> &BTreeMap<String, ArchEntry> {
        &self.arch
    }

    /// Assemble the legacy [`PlatformManifest`] view (the `[build.zenoh]`
    /// payloads keyed by platform name, `inherits` preserved) so the
    /// existing `for_platform` inheritance/merge logic — and every
    /// downstream consumer of `ResolvedPlatform` — keeps working
    /// unchanged.
    pub fn as_platform_manifest(&self) -> PlatformManifest {
        let mut platform = BTreeMap::new();
        for (name, file) in &self.files {
            // phase-347 W6 — `[build.<component>]` by key. Still "zenoh" here: this
            // assembles the zenoh-pico system-layer view. A second component
            // reads its own key without a schema change.
            let mut entry = file.build.get("zenoh").cloned().unwrap_or_default();
            // `inherits` lives at file top level in the new format; the
            // legacy resolver reads it from the entry.
            if entry.inherits.is_none() {
                entry.inherits = file.inherits.clone();
            }
            platform.insert(name.clone(), entry);
        }
        PlatformManifest {
            platform,
            arch: self.arch.clone(),
        }
    }

    /// The directory a platform NAME resolves to, following `[names]` aliases.
    ///
    /// phase-349 W1. A file's directory name always resolves to itself, so a
    /// tree whose files declare no `names` behaves exactly as before. Aliases
    /// are additive on top.
    ///
    /// Returns the input unchanged when nothing claims it, so the caller's
    /// existing "unknown platform" error still names what the user asked for
    /// rather than something this function invented.
    pub fn resolve_alias<'a>(&'a self, name: &'a str) -> &'a str {
        if self.files.contains_key(name) {
            return name;
        }
        self.files
            .iter()
            .find(|(_, f)| f.names.iter().any(|n| n == name))
            .map(|(dir, _)| dir.as_str())
            .unwrap_or(name)
    }

    /// Every name the tree answers to, directory names and aliases alike.
    /// Used to make an unknown-platform error list the real options.
    pub fn all_names(&self) -> Vec<String> {
        let mut out: Vec<String> = self
            .files
            .iter()
            .flat_map(|(dir, f)| std::iter::once(dir.clone()).chain(f.names.iter().cloned()))
            .collect();
        out.sort();
        out.dedup();
        out
    }

    /// Walk the `inherits` chain (child-first list: `[name, parent, …]`).
    ///
    /// Alias resolution happens HERE, at the one point every public lookup
    /// (`capabilities`, `resolve_tx`, `capability_check`) funnels through —
    /// rather than at each caller, which is how half of them would end up
    /// alias-blind.
    fn chain(&self, name: &str) -> Result<Vec<&PlatformConfigFile>, ConfigError> {
        let mut out = Vec::new();
        let mut seen = std::collections::BTreeSet::new();
        let mut cur = Some(self.resolve_alias(name).to_string());
        while let Some(n) = cur {
            if !seen.insert(n.clone()) {
                return Err(ConfigError::InheritsCycle { name: n });
            }
            let file = self
                .files
                .get(&n)
                .ok_or_else(|| ConfigError::UnknownPlatform {
                    name: n.clone(),
                    root: self.root.display().to_string(),
                })?;
            cur = file
                .inherits
                .as_deref()
                .map(|p| self.resolve_alias(p).to_string());
            out.push(file);
        }
        Ok(out)
    }

    /// Resolve one platform's capabilities (inherits-merged, child wins).
    pub fn capabilities(&self, name: &str) -> Result<BTreeMap<String, bool>, ConfigError> {
        let chain = self.chain(name)?;
        let mut caps = BTreeMap::new();
        // Parent-first application so the child overrides.
        for file in chain.iter().rev() {
            for (k, v) in &file.capabilities {
                caps.insert(k.clone(), *v);
            }
        }
        Ok(caps)
    }

    /// Resolve one platform's `[knobs]` (inherits-merged, child wins,
    /// field-level).
    fn platform_tx_knobs(&self, name: &str) -> Result<TxKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut tx = TxKnobs::default();
        for file in chain.iter().rev() {
            let t = &file.knobs.zenoh.tx;
            if t.batch.is_some() {
                tx.batch = t.batch;
            }
            if t.split_lock.is_some() {
                tx.split_lock = t.split_lock;
            }
            if t.flush_ms.is_some() {
                tx.flush_ms = t.flush_ms;
            }
        }
        Ok(tx)
    }

    /// Same inheritance walk as [`Self::platform_tx_knobs`], for the transport
    /// tenant: a child platform overrides its parent field by field, and an
    /// unset field defers rather than resetting to the default.
    fn platform_transport_knobs(&self, name: &str) -> Result<TransportKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut t = TransportKnobs::default();
        for file in chain.iter().rev() {
            let f = &file.knobs.transport;
            if f.kind.is_some() {
                t.kind = f.kind.clone();
            }
            if f.endpoint.is_some() {
                t.endpoint = f.endpoint.clone();
            }
        }
        Ok(t)
    }

    /// Resolve the `zenoh.tx` knob set for `platform`, applying the full
    /// ladder: builtin < platform < `board` deltas < `env` overrides.
    ///
    /// `env` is an accessor (injected for tests): `env(name)` returns the
    /// raw env value if SET (tri-state front-end — a set `"0"` overrides
    /// an on-default).
    pub fn resolve_tx(
        &self,
        platform: &str,
        board: Option<&TxKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
    ) -> Result<ResolvedTxKnobs, ConfigError> {
        let plat = self.platform_tx_knobs(platform)?;

        fn rung<T: Copy>(builtin: T, plat: Option<T>, board: Option<T>) -> (T, KnobSource) {
            match (board, plat) {
                (Some(v), _) => (v, KnobSource::Board),
                (None, Some(v)) => (v, KnobSource::Platform),
                (None, None) => (builtin, KnobSource::Builtin),
            }
        }

        let (mut batch, mut batch_src) =
            rung(BUILTIN_TX_BATCH, plat.batch, board.and_then(|b| b.batch));
        let (mut split, mut split_src) = rung(
            BUILTIN_TX_SPLIT_LOCK,
            plat.split_lock,
            board.and_then(|b| b.split_lock),
        );
        let (mut flush, mut flush_src) = rung(
            BUILTIN_TX_FLUSH_MS,
            plat.flush_ms,
            board.and_then(|b| b.flush_ms),
        );

        // Env front-end — top rung, tri-state.
        if let Some(v) = env("ZPICO_TX_BATCH") {
            batch = v.trim().parse::<u64>().map(|n| n != 0).unwrap_or(false);
            batch_src = KnobSource::Env;
        }
        if let Some(v) = env("ZPICO_TX_SPLIT_LOCK") {
            split = v.trim().parse::<u64>().map(|n| n != 0).unwrap_or(false);
            split_src = KnobSource::Env;
        }
        if let Some(v) = env("ZPICO_TX_BATCH_FLUSH_MS")
            && let Ok(n) = v.trim().parse::<u64>()
        {
            flush = n;
            flush_src = KnobSource::Env;
        }

        Ok(ResolvedTxKnobs {
            batch: Resolved {
                value: batch,
                source: batch_src,
            },
            split_lock: Resolved {
                value: split,
                source: split_src,
            },
            flush_ms: Resolved {
                value: flush,
                source: flush_src,
            },
        })
    }

    /// phase-400 W6 — resolve the executor sizing tenant.
    ///
    /// Same four-rung ladder as every other tenant. `defaults` carries the
    /// crate's built-in values so this module does not duplicate them: the
    /// numbers stay in `nros-node/build.rs`, which is the one place that knows
    /// how the arena is derived from them.
    ///
    /// Each knob keeps its existing env name, so migrating a knob changes no
    /// call site — it only adds the two rungs it never had.
    /// The `[knobs.params]` a platform's chain declares, merged nearest-wins.
    fn platform_param_knobs(&self, name: &str) -> Result<ParamKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut out = ParamKnobs::default();
        for file in chain.iter().rev() {
            let p = &file.knobs.params;
            for (dst, src) in [
                (&mut out.max_parameters, p.max_parameters),
                (&mut out.max_param_name_len, p.max_param_name_len),
                (&mut out.max_string_value_len, p.max_string_value_len),
                (&mut out.max_array_len, p.max_array_len),
                (&mut out.max_byte_array_len, p.max_byte_array_len),
            ] {
                if src.is_some() {
                    *dst = src;
                }
            }
        }
        Ok(out)
    }

    /// The `[knobs.params]` rungs, without resolution — for a build script that
    /// owns the defaults and its own env/Kconfig front-end.
    fn platform_rmw_knobs(&self, name: &str) -> Result<RmwKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut out = RmwKnobs::default();
        for file in chain.iter().rev() {
            let r = &file.knobs.rmw;
            for (dst, src) in [
                (&mut out.max_backends, r.max_backends),
                (&mut out.max_nodes, r.max_nodes),
                (&mut out.message_info_slots, r.message_info_slots),
            ] {
                if src.is_some() {
                    *dst = src;
                }
            }
        }
        Ok(out)
    }

    fn platform_net_knobs(&self, name: &str) -> Result<NetKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut out = NetKnobs::default();
        for file in chain.iter().rev() {
            let n = &file.knobs.net;
            for (dst, src) in [
                (&mut out.max_sockets, n.max_sockets),
                (&mut out.max_udp_sockets, n.max_udp_sockets),
                (&mut out.buffer_size, n.buffer_size),
                (&mut out.connect_timeout_ms, n.connect_timeout_ms),
                (&mut out.socket_timeout_ms, n.socket_timeout_ms),
            ] {
                if src.is_some() {
                    *dst = src;
                }
            }
        }
        Ok(out)
    }

    fn platform_runtime_knobs(&self, name: &str) -> Result<RuntimeKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut out = RuntimeKnobs::default();
        for file in chain.iter().rev() {
            let r = &file.knobs.runtime;
            for (dst, src) in [
                (&mut out.max_components, r.max_components),
                (&mut out.component_slot_bytes, r.component_slot_bytes),
                (&mut out.max_class_instances, r.max_class_instances),
                (&mut out.max_cell_entities, r.max_cell_entities),
                (&mut out.let_buffer_size, r.let_buffer_size),
            ] {
                if src.is_some() {
                    *dst = src;
                }
            }
        }
        Ok(out)
    }

    fn platform_wire_knobs(&self, name: &str) -> Result<WireKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut out = WireKnobs::default();
        for file in chain.iter().rev() {
            let w = &file.knobs.zenoh.wire;
            for (dst, src) in [
                (&mut out.batch_unicast_size, w.batch_unicast_size),
                (&mut out.batch_multicast_size, w.batch_multicast_size),
                (&mut out.frag_max_size, w.frag_max_size),
                (&mut out.get_reply_buf_size, w.get_reply_buf_size),
                (&mut out.get_poll_interval_ms, w.get_poll_interval_ms),
            ] {
                if src.is_some() {
                    *dst = src;
                }
            }
        }
        Ok(out)
    }

    /// phase-400 W6 — resolve the zenoh WIRE tenant over the RFC-0049 ladder.
    ///
    /// `defaults` carries the CALLER's builtins because they are computed, not
    /// constant: `nros-zpico-build` picks a batch and fragmentation size from
    /// the platform's transport before any descriptor is consulted. The ladder
    /// still owns the rungs above them.
    pub fn resolve_wire(
        &self,
        platform: &str,
        board: Option<&WireKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
        defaults: &[(&'static str, usize)],
    ) -> Result<Vec<(&'static str, ResolvedUsize)>, ConfigError> {
        let plat = self.platform_wire_knobs(platform)?;
        let pick = |k: &WireKnobs, name: &str| match name {
            "batch_unicast_size" => k.batch_unicast_size,
            "batch_multicast_size" => k.batch_multicast_size,
            "frag_max_size" => k.frag_max_size,
            "get_reply_buf_size" => k.get_reply_buf_size,
            "get_poll_interval_ms" => k.get_poll_interval_ms,
            _ => None,
        };
        let mut out = Vec::new();
        for (name, builtin) in defaults {
            let (mut value, mut source) =
                match (board.and_then(|b| pick(b, name)), pick(&plat, name)) {
                    (Some(v), _) => (v, KnobSource::Board),
                    (None, Some(v)) => (v, KnobSource::Platform),
                    (None, None) => (*builtin, KnobSource::Builtin),
                };
            let env_key = wire_env_key(name);
            if let Some(raw) = env(env_key)
                && let Ok(n) = raw.trim().parse::<usize>()
            {
                value = n;
                source = KnobSource::Env;
            }
            out.push((
                *name,
                ResolvedUsize {
                    value,
                    source,
                    env_key,
                },
            ));
        }
        Ok(out)
    }

    fn platform_xrce_knobs(&self, name: &str) -> Result<XrceKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut out = XrceKnobs::default();
        for file in chain.iter().rev() {
            let x = &file.knobs.xrce;
            for (dst, src) in [
                (&mut out.custom_transport_mtu, x.custom_transport_mtu),
                (&mut out.stream_history, x.stream_history),
            ] {
                if src.is_some() {
                    *dst = src;
                }
            }
        }
        Ok(out)
    }

    /// The `[knobs.xrce]` rungs for a platform, inherits chain applied.
    pub fn platform_xrce_rungs(&self, platform: &str) -> Result<XrceKnobs, ConfigError> {
        self.platform_xrce_knobs(platform)
    }

    fn platform_zenoh_limit_knobs(&self, name: &str) -> Result<ZenohLimitKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut out = ZenohLimitKnobs::default();
        for file in chain.iter().rev() {
            let z = &file.knobs.zenoh.limits;
            for (dst, src) in [
                (&mut out.keyexpr_string_size, z.keyexpr_string_size),
                (&mut out.service_timeout_ms, z.service_timeout_ms),
                (&mut out.subscriber_ring_depth, z.subscriber_ring_depth),
            ] {
                if src.is_some() {
                    *dst = src;
                }
            }
        }
        Ok(out)
    }

    /// The `[knobs.zenoh.limits]` rungs for a platform, inherits chain applied.
    pub fn platform_zenoh_limit_rungs(
        &self,
        platform: &str,
    ) -> Result<ZenohLimitKnobs, ConfigError> {
        self.platform_zenoh_limit_knobs(platform)
    }

    /// The `[knobs.zenoh.wire]` rungs for a platform, inherits chain applied.
    pub fn platform_wire_rungs(&self, platform: &str) -> Result<WireKnobs, ConfigError> {
        self.platform_wire_knobs(platform)
    }

    /// The `[knobs.runtime]` rungs for a platform, inherits chain applied.
    pub fn platform_runtime_rungs(&self, platform: &str) -> Result<RuntimeKnobs, ConfigError> {
        self.platform_runtime_knobs(platform)
    }

    /// phase-400 W6 — resolve the runtime tenant over the RFC-0049 ladder.
    pub fn resolve_runtime(
        &self,
        platform: &str,
        board: Option<&RuntimeKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
        defaults: &[(&'static str, usize)],
    ) -> Result<Vec<(&'static str, ResolvedUsize)>, ConfigError> {
        let plat = self.platform_runtime_knobs(platform)?;
        let pick = |k: &RuntimeKnobs, name: &str| match name {
            "max_components" => k.max_components,
            "component_slot_bytes" => k.component_slot_bytes,
            "max_class_instances" => k.max_class_instances,
            "max_cell_entities" => k.max_cell_entities,
            "let_buffer_size" => k.let_buffer_size,
            _ => None,
        };
        let mut out = Vec::new();
        for (name, builtin) in defaults {
            let (mut value, mut source) =
                match (board.and_then(|b| pick(b, name)), pick(&plat, name)) {
                    (Some(v), _) => (v, KnobSource::Board),
                    (None, Some(v)) => (v, KnobSource::Platform),
                    (None, None) => (*builtin, KnobSource::Builtin),
                };
            let env_key = runtime_env_key(name);
            if let Some(raw) = env(env_key)
                && let Ok(n) = raw.trim().parse::<usize>()
            {
                value = n;
                source = KnobSource::Env;
            }
            out.push((
                *name,
                ResolvedUsize {
                    value,
                    source,
                    env_key,
                },
            ));
        }
        Ok(out)
    }

    /// The `[knobs.net]` rungs for a platform, inherits chain applied.
    pub fn platform_net_rungs(&self, platform: &str) -> Result<NetKnobs, ConfigError> {
        self.platform_net_knobs(platform)
    }

    /// phase-400 W6 — resolve the net tenant over the RFC-0049 ladder.
    pub fn resolve_net(
        &self,
        platform: &str,
        board: Option<&NetKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
        defaults: &[(&'static str, usize)],
    ) -> Result<Vec<(&'static str, ResolvedUsize)>, ConfigError> {
        let plat = self.platform_net_knobs(platform)?;
        let pick = |k: &NetKnobs, name: &str| match name {
            "max_sockets" => k.max_sockets,
            "max_udp_sockets" => k.max_udp_sockets,
            "buffer_size" => k.buffer_size,
            "connect_timeout_ms" => k.connect_timeout_ms,
            "socket_timeout_ms" => k.socket_timeout_ms,
            _ => None,
        };
        let mut out = Vec::new();
        for (name, builtin) in defaults {
            let (mut value, mut source) =
                match (board.and_then(|b| pick(b, name)), pick(&plat, name)) {
                    (Some(v), _) => (v, KnobSource::Board),
                    (None, Some(v)) => (v, KnobSource::Platform),
                    (None, None) => (*builtin, KnobSource::Builtin),
                };
            let env_key = net_env_key(name);
            if let Some(raw) = env(env_key)
                && let Ok(n) = raw.trim().parse::<usize>()
            {
                value = n;
                source = KnobSource::Env;
            }
            out.push((
                *name,
                ResolvedUsize {
                    value,
                    source,
                    env_key,
                },
            ));
        }
        Ok(out)
    }

    /// The `[knobs.rmw]` rungs for a platform, inherits chain applied.
    pub fn platform_rmw_rungs(&self, platform: &str) -> Result<RmwKnobs, ConfigError> {
        self.platform_rmw_knobs(platform)
    }

    pub fn platform_param_rungs(&self, platform: &str) -> Result<ParamKnobs, ConfigError> {
        self.platform_param_knobs(platform)
    }

    /// phase-400 W6 — resolve the RMW tenant over the RFC-0049 ladder.
    ///
    /// Mirrors [`Self::resolve_params`]; the build script keeps its own range
    /// checks, which belong to the array it carves rather than to the ladder.
    pub fn resolve_rmw(
        &self,
        platform: &str,
        board: Option<&RmwKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
        defaults: &[(&'static str, usize)],
    ) -> Result<Vec<(&'static str, ResolvedUsize)>, ConfigError> {
        let plat = self.platform_rmw_knobs(platform)?;
        let pick = |k: &RmwKnobs, name: &str| match name {
            "max_backends" => k.max_backends,
            "max_nodes" => k.max_nodes,
            "message_info_slots" => k.message_info_slots,
            _ => None,
        };
        let mut out = Vec::new();
        for (name, builtin) in defaults {
            let (mut value, mut source) =
                match (board.and_then(|b| pick(b, name)), pick(&plat, name)) {
                    (Some(v), _) => (v, KnobSource::Board),
                    (None, Some(v)) => (v, KnobSource::Platform),
                    (None, None) => (*builtin, KnobSource::Builtin),
                };
            let env_key = rmw_env_key(name);
            if let Some(raw) = env(env_key)
                && let Ok(n) = raw.trim().parse::<usize>()
            {
                value = n;
                source = KnobSource::Env;
            }
            out.push((
                *name,
                ResolvedUsize {
                    value,
                    source,
                    env_key,
                },
            ));
        }
        Ok(out)
    }

    /// phase-400 W6 — resolve the parameter tenant over the RFC-0049 ladder.
    pub fn resolve_params(
        &self,
        platform: &str,
        board: Option<&ParamKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
        defaults: &[(&'static str, usize)],
    ) -> Result<Vec<(&'static str, ResolvedUsize)>, ConfigError> {
        let plat = self.platform_param_knobs(platform)?;
        let pick = |k: &ParamKnobs, name: &str| match name {
            "max_parameters" => k.max_parameters,
            "max_param_name_len" => k.max_param_name_len,
            "max_string_value_len" => k.max_string_value_len,
            "max_array_len" => k.max_array_len,
            "max_byte_array_len" => k.max_byte_array_len,
            _ => None,
        };
        let mut out = Vec::new();
        for (name, builtin) in defaults {
            let (mut value, mut source) =
                match (board.and_then(|b| pick(b, name)), pick(&plat, name)) {
                    (Some(v), _) => (v, KnobSource::Board),
                    (None, Some(v)) => (v, KnobSource::Platform),
                    (None, None) => (*builtin, KnobSource::Builtin),
                };
            let env_key = param_env_key(name);
            if let Some(raw) = env(env_key)
                && let Ok(n) = raw.trim().parse::<usize>()
            {
                value = n;
                source = KnobSource::Env;
            }
            out.push((
                *name,
                ResolvedUsize {
                    value,
                    source,
                    env_key,
                },
            ));
        }
        Ok(out)
    }

    /// The `[knobs.memory]` a platform's chain declares, merged nearest-wins.
    fn platform_memory_knobs(&self, name: &str) -> Result<MemoryKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut out = MemoryKnobs::default();
        for file in chain.iter().rev() {
            let m = &file.knobs.memory;
            if m.heap_bytes.is_some() {
                out.heap_bytes = m.heap_bytes;
            }
            if m.app_stack_bytes.is_some() {
                out.app_stack_bytes = m.app_stack_bytes;
            }
        }
        Ok(out)
    }

    /// The `[knobs.memory]` rungs, without resolution — for a build script that
    /// owns the defaults and its own env/Kconfig front-end. Same shape as
    /// [`Self::platform_executor_rungs`], and the same reason.
    pub fn platform_memory_rungs(&self, platform: &str) -> Result<MemoryKnobs, ConfigError> {
        self.platform_memory_knobs(platform)
    }

    /// phase-400 W6 — resolve the memory tenant over the RFC-0049 ladder:
    /// builtin < platform toml < board toml < env front-end.
    ///
    /// `defaults` are passed in, as for the executor tenant: the built-in for
    /// a FreeRTOS heap lives in the board crate that sizes it, and an absent
    /// platform tree must change nothing.
    pub fn resolve_memory(
        &self,
        platform: &str,
        board: Option<&MemoryKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
        defaults: &[(&'static str, usize)],
    ) -> Result<Vec<(&'static str, ResolvedUsize)>, ConfigError> {
        let plat = self.platform_memory_knobs(platform)?;
        Ok(self.resolve_memory_from(platform, &plat, board, env, defaults))
    }

    /// The memory ladder over an ALREADY-RESOLVED platform rung.
    ///
    /// Split out so the build side can supply an empty rung for a platform that
    /// declares no descriptor (see `BuildRungs::or_builtin_rungs`) without
    /// re-implementing the ladder, while `nros config explain` keeps the strict
    /// lookup above — a typo in `--platform` should still be an error there,
    /// and silently printing builtins for it would be the wrong answer.
    pub fn resolve_memory_from(
        &self,
        platform: &str,
        plat: &MemoryKnobs,
        board: Option<&MemoryKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
        defaults: &[(&'static str, usize)],
    ) -> Vec<(&'static str, ResolvedUsize)> {
        let pick = |k: &MemoryKnobs, name: &str| match name {
            "heap_bytes" => k.heap_bytes,
            "app_stack_bytes" => k.app_stack_bytes,
            _ => None,
        };
        let mut out = Vec::new();
        for (name, builtin) in defaults {
            let (mut value, mut source) =
                match (board.and_then(|b| pick(b, name)), pick(plat, name)) {
                    (Some(v), _) => (v, KnobSource::Board),
                    (None, Some(v)) => (v, KnobSource::Platform),
                    (None, None) => (*builtin, KnobSource::Builtin),
                };
            let env_key = memory_env_key_for(platform, name);
            if let Some(raw) = env(env_key)
                && let Ok(n) = raw.trim().parse::<usize>()
            {
                // The front-ends disagree about units and the ladder does not:
                // a KiB spelling is multiplied here, once, beside the table
                // that records which spelling each knob has.
                value = if memory_env_is_kib_for(platform, name) {
                    n * 1024
                } else {
                    n
                };
                source = KnobSource::Env;
            }
            out.push((
                *name,
                ResolvedUsize {
                    value,
                    source,
                    env_key,
                },
            ));
        }
        out
    }

    /// The `[knobs.executor]` a platform's chain declares, merged nearest-wins.
    ///
    /// Public because a BUILD SCRIPT needs the rungs without the resolution:
    /// `nros-node/build.rs` owns the defaults (two are derived) and its own
    /// env/Kconfig front-end, so it wants "what do the platform and board
    /// say", not "what is the final answer".
    pub fn platform_executor_rungs(&self, platform: &str) -> Result<ExecutorKnobs, ConfigError> {
        self.platform_executor_knobs(platform)
    }

    pub fn resolve_executor(
        &self,
        platform: &str,
        board: Option<&ExecutorKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
        defaults: &[(&'static str, usize)],
    ) -> Result<Vec<(&'static str, ResolvedUsize)>, ConfigError> {
        let plat = self.platform_executor_knobs(platform)?;

        let pick = |name: &'static str| -> Option<usize> {
            match name {
                "max_cbs" => plat.max_cbs,
                "max_sc" => plat.max_sc,
                "max_nodes" => plat.max_nodes,
                "max_shutdown_cbs" => plat.max_shutdown_cbs,
                "action_clients" => plat.action_clients,
                "arena_size" => plat.arena_size,
                "subscription_buffer_size" => plat.subscription_buffer_size,
                "param_service_buffer_size" => plat.param_service_buffer_size,
                _ => None,
            }
        };
        let pick_board = |name: &'static str| -> Option<usize> {
            let b = board?;
            match name {
                "max_cbs" => b.max_cbs,
                "max_sc" => b.max_sc,
                "max_nodes" => b.max_nodes,
                "max_shutdown_cbs" => b.max_shutdown_cbs,
                "action_clients" => b.action_clients,
                "arena_size" => b.arena_size,
                "subscription_buffer_size" => b.subscription_buffer_size,
                "param_service_buffer_size" => b.param_service_buffer_size,
                _ => None,
            }
        };

        let mut out = Vec::new();
        for (name, builtin) in defaults {
            let env_key = executor_env_key(name);
            let (value, source) = match (
                env(env_key).and_then(|v| v.trim().parse::<usize>().ok()),
                pick_board(name),
                pick(name),
            ) {
                (Some(v), _, _) => (v, KnobSource::Env),
                (None, Some(v), _) => (v, KnobSource::Board),
                (None, None, Some(v)) => (v, KnobSource::Platform),
                (None, None, None) => (*builtin, KnobSource::Builtin),
            };
            out.push((
                *name,
                ResolvedUsize {
                    value,
                    source,
                    env_key,
                },
            ));
        }
        Ok(out)
    }

    fn platform_executor_knobs(&self, name: &str) -> Result<ExecutorKnobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut k = ExecutorKnobs::default();
        for file in chain.iter().rev() {
            let f = &file.knobs.executor;
            macro_rules! take {
                ($($fld:ident),*) => { $( if f.$fld.is_some() { k.$fld = f.$fld; } )* };
            }
            take!(
                max_cbs,
                max_sc,
                max_nodes,
                max_shutdown_cbs,
                action_clients,
                arena_size,
                subscription_buffer_size,
                param_service_buffer_size
            );
        }
        Ok(k)
    }

    /// phase-400 W3 / RFC-0086 D1+D2 — resolve the transport tenant and the
    /// knobs it implies.
    ///
    /// Three verbs, with the strengths the surveyed prior art settles:
    ///
    /// * `exactly-one-of` (Gentoo `REQUIRED_USE`) — `kind` is one of
    ///   [`TRANSPORT_KINDS`]. An unrecognised value is an error, not a silent
    ///   pass-through: it would leave every implication unapplied and build an
    ///   image with the wrong links enabled.
    /// * `requires` (hard) — the platform must declare the capability the
    ///   transport needs. Failure names both the platform and the asked-for
    ///   kind, at CONFIG time, rather than surfacing later as a link error
    ///   against `AF_INET`.
    /// * `implies` (weak, Kconfig `imply` strength) — the dependent link and
    ///   driver knobs. `env_overrides` lets a higher rung win; the override is
    ///   recorded, never silently dropped.
    pub fn resolve_transport(
        &self,
        platform: &str,
        board: Option<&TransportKnobs>,
        env: &dyn Fn(&str) -> Option<String>,
    ) -> Result<ResolvedTransport, ConfigError> {
        let plat = self.platform_transport_knobs(platform)?;

        let (kind, kind_src) = match (
            env("NROS_TRANSPORT_KIND"),
            board.and_then(|b| b.kind.clone()),
            plat.kind.clone(),
        ) {
            (Some(v), _, _) => (v.trim().to_string(), KnobSource::Env),
            (None, Some(v), _) => (v, KnobSource::Board),
            (None, None, Some(v)) => (v, KnobSource::Platform),
            (None, None, None) => (BUILTIN_TRANSPORT_KIND.to_string(), KnobSource::Builtin),
        };

        if !TRANSPORT_KINDS.contains(&kind.as_str()) {
            return Err(ConfigError::UnknownTransport {
                platform: platform.to_string(),
                kind,
                known: TRANSPORT_KINDS.join(", "),
            });
        }

        let (endpoint, endpoint_src) = match (
            env("NROS_TRANSPORT_ENDPOINT"),
            board.and_then(|b| b.endpoint.clone()),
            plat.endpoint.clone(),
        ) {
            (Some(v), _, _) => (Some(v), KnobSource::Env),
            (None, Some(v), _) => (Some(v), KnobSource::Board),
            (None, None, Some(v)) => (Some(v), KnobSource::Platform),
            (None, None, None) => (None, KnobSource::Builtin),
        };

        // `requires` — hard. A capability is a FACT declared by the platform;
        // policy that contradicts fact must not ship (RFC-0049), and for a
        // transport that means failing here rather than at link time.
        let caps = self.capabilities(platform)?;
        let needed = match kind.as_str() {
            "tcp" | "udp" => Some("ip_stack"),
            "serial" => Some("serial"),
            _ => None,
        };
        // Absent is NOT the same as false. `[capabilities]` is an open, young
        // vocabulary -- only one of the seven in-tree platform files declared
        // anything when this landed -- so an UNDECLARED fact means "nobody has
        // described this platform yet" and must not fail a build that works.
        // An EXPLICIT `false` is a described impossibility and is hard.
        //
        // The undeclared case is reported by `transport_warnings` rather than
        // swallowed, so the gap is visible and gets closed by declaration.
        if let Some(cap) = needed
            && caps.get(cap) == Some(&false)
        {
            return Err(ConfigError::TransportCapabilityMissing {
                platform: platform.to_string(),
                kind: kind.clone(),
                capability: cap.to_string(),
                source: kind_src.as_str().to_string(),
            });
        }

        // `implies` — weak. Every knob a transport choice settles, so no image
        // has to hand-write them. The env front-end still wins; when it does,
        // the implication is marked overridden rather than discarded.
        let rule = format!("transport.kind={kind}");
        let mut implied = Vec::new();
        let mut imply = |knob: &str, value: bool, env_key: &str| {
            let overridden_by = env(env_key).and_then(|v| {
                let want = v.trim().parse::<u64>().map(|n| n != 0).unwrap_or(false);
                (want != value).then_some(KnobSource::Env)
            });
            implied.push(Implied {
                knob: knob.to_string(),
                value,
                rule: rule.clone(),
                overridden_by,
            });
        };

        let serial = kind == "serial";
        imply("links.tcp", kind == "tcp", "NROS_ZENOH_LINK_TCP");
        imply("links.udp", kind == "udp", "NROS_ZENOH_LINK_UDP_UNICAST");
        imply("links.serial", serial, "NROS_ZENOH_LINK_SERIAL");
        // The MAC/MDIO/PHY drivers are `default y` behind devicetree nodes the
        // board has enabled, so they arrive on their own and must be turned off
        // by name. This is the fifteen-line hand-written block RFC-0086 exists
        // to delete.
        imply("drivers.ethernet", !serial, "NROS_DRIVER_ETHERNET");
        imply("drivers.phy", !serial, "NROS_DRIVER_PHY");
        imply("drivers.mdio", !serial, "NROS_DRIVER_MDIO");
        imply("net.ip_stack", !serial, "NROS_NET_IP_STACK");

        Ok(ResolvedTransport {
            kind: Resolved {
                value: kind,
                source: kind_src,
            },
            endpoint: Resolved {
                value: endpoint,
                source: endpoint_src,
            },
            implied,
        })
    }

    /// The merged `[knobs]` table for `platform`, following `inherits`.
    /// Exposed so a caller that HAS loaded the rmw descriptors can run
    /// [`Knobs::unknown_backend_keys`] against the real backend list.
    pub fn knobs_for(&self, name: &str) -> Result<Knobs, ConfigError> {
        let chain = self.chain(name)?;
        let mut out = Knobs::default();
        for file in chain.iter().rev() {
            if file.knobs.zenoh.tx.batch.is_some()
                || file.knobs.zenoh.tx.split_lock.is_some()
                || file.knobs.zenoh.tx.flush_ms.is_some()
            {
                out.zenoh = file.knobs.zenoh.clone();
            }
            if file.knobs.transport.kind.is_some() || file.knobs.transport.endpoint.is_some() {
                out.transport = file.knobs.transport.clone();
            }
            for (k, v) in &file.knobs.backends {
                out.backends.insert(k.clone(), v.clone());
            }
        }
        Ok(out)
    }

    /// phase-400 W3 — facts this transport choice relied on but the platform
    /// never declared. Not an error (see `resolve_transport`), but printed, so
    /// an undescribed platform is visible rather than silently assumed.
    pub fn transport_warnings(
        &self,
        platform: &str,
        t: &ResolvedTransport,
    ) -> Result<Vec<String>, ConfigError> {
        let caps = self.capabilities(platform)?;
        let needed = match t.kind.value.as_str() {
            "tcp" | "udp" => Some("ip_stack"),
            "serial" => Some("serial"),
            _ => None,
        };
        let mut out = Vec::new();
        if let Some(cap) = needed
            && !caps.contains_key(cap)
        {
            out.push(format!(
                "platform `{platform}`: transport.kind = `{}` needs capabilities.{cap}, which \
                 this platform's nros-platform.toml does not declare either way — permitted, but \
                 the fact should be stated",
                t.kind.value
            ));
        }
        Ok(out)
    }

    /// RFC-0049 capability cross-check: policy that contradicts fact is
    /// downgraded, never silently shipped. Returns warning lines (the
    /// build script prints them as `cargo:warning=`).
    pub fn capability_check(
        &self,
        platform: &str,
        tx: &mut ResolvedTxKnobs,
    ) -> Result<Vec<String>, ConfigError> {
        let caps = self.capabilities(platform)?;
        let mut warnings = Vec::new();
        let threads = caps.get("threads").copied().unwrap_or(false);
        if tx.split_lock.value && !threads {
            warnings.push(format!(
                "platform `{platform}`: knobs.zenoh.tx.split_lock = true (from {}) but \
                 capabilities.threads is not true — split locking needs a flush thread; \
                 downgrading split_lock to off",
                tx.split_lock.source.as_str()
            ));
            tx.split_lock = Resolved {
                value: false,
                source: KnobSource::Builtin,
            };
        }
        Ok(warnings)
    }
}

/// One board package's knob deltas, read from the `[knobs]` table of its
/// existing `nros-board.toml` (RFC-0042 descriptor — the rest of that
/// file is parsed elsewhere and unknown keys there are NOT this module's
/// concern, so this parse is deliberately tolerant of sibling tables).
#[derive(Debug, Default, Deserialize)]
pub struct BoardKnobsFile {
    #[serde(default)]
    pub capabilities: BTreeMap<String, bool>,
    #[serde(default)]
    pub knobs: Knobs,
    // The rest of nros-board.toml (board descriptor tables) is ignored
    // here — parsed by the board registry, not the knob ladder.
    #[serde(flatten)]
    _rest: BTreeMap<String, toml::Value>,
}

impl BoardKnobsFile {
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let text = fs::read_to_string(path).map_err(|e| ConfigError::Io {
            path: path.display().to_string(),
            source: e,
        })?;
        toml::from_str(&text).map_err(|e| ConfigError::Parse {
            path: path.display().to_string(),
            source: e,
        })
    }
}

#[cfg(test)]
mod tests {
    /// The zenoh WIRE ladder: builtin < platform < board < env.
    ///
    /// The `nros-zpico-build` side of this tenant writes a C header from an
    /// example build tree, which made an end-to-end probe unreliable to observe
    /// — so the RESOLVER is pinned here instead, and the build script's job is
    /// reduced to five straight-line assignments that mirror the tx trio two
    /// lines above them.
    #[test]
    fn the_wire_ladder_resolves_builtin_platform_board_and_env() {
        let dir = tempfile::tempdir().expect("tempdir");
        std::fs::create_dir_all(dir.path().join("posix")).unwrap();
        std::fs::write(
            dir.path().join("posix/nros-platform.toml"),
            "[knobs.zenoh.wire]\nfrag_max_size = 777\nget_reply_buf_size = 1234\n",
        )
        .unwrap();
        let tree = PlatformsTree::load_search_path(&[dir.path().to_path_buf()]).expect("tree");
        let defaults = [
            ("frag_max_size", 2048usize),
            ("get_reply_buf_size", 4096),
            ("get_poll_interval_ms", 10),
        ];

        // platform rung over the caller's computed builtins
        let got = tree
            .resolve_wire("posix", None, &|_| None, &defaults)
            .expect("resolve");
        let by = |n: &str| got.iter().find(|(k, _)| *k == n).unwrap().1.clone();
        assert_eq!(by("frag_max_size").value, 777);
        assert_eq!(by("frag_max_size").source, KnobSource::Platform);
        // a knob the platform does not name keeps the CALLER's builtin, which is
        // the whole reason `defaults` is a parameter: it is computed per
        // transport, not a constant.
        assert_eq!(by("get_poll_interval_ms").value, 10);
        assert_eq!(by("get_poll_interval_ms").source, KnobSource::Builtin);

        // board outranks platform
        let board = WireKnobs {
            frag_max_size: Some(99),
            ..Default::default()
        };
        let got = tree
            .resolve_wire("posix", Some(&board), &|_| None, &defaults)
            .expect("resolve");
        let hit = got.iter().find(|(k, _)| *k == "frag_max_size").unwrap();
        assert_eq!(hit.1.value, 99);
        assert_eq!(hit.1.source, KnobSource::Board);

        // env outranks both
        let got = tree
            .resolve_wire(
                "posix",
                Some(&board),
                &|k| (k == "ZPICO_FRAG_MAX_SIZE").then(|| "555".to_string()),
                &defaults,
            )
            .expect("resolve");
        let hit = got.iter().find(|(k, _)| *k == "frag_max_size").unwrap();
        assert_eq!(hit.1.value, 555);
        assert_eq!(hit.1.source, KnobSource::Env);
    }

    /// A platform with NO descriptor resolves to builtins; a BROKEN one does not.
    ///
    /// The phase-400 W6 regression this pins killed every image of the three
    /// platforms that declare no `nros-platform.toml` — `threadx-linux`,
    /// `esp32`, `zephyr` — by panicking in a build script. Issue 0979 fixed the
    /// search ROOT being empty; this is the other half, and the two look
    /// identical in a log (`unknown platform \`x\`: no <root>/x/…`), which is
    /// why the second one survived the first fix.
    ///
    /// Both directions, because the tolerant arm must not swallow a real error:
    /// an absent file is NO answer (builtins are correct), a malformed file is
    /// a WRONG answer (still fatal).
    #[test]
    fn an_absent_descriptor_is_unknown_platform_and_a_broken_one_is_not() {
        let dir = tempfile::tempdir().expect("tempdir");
        let root = dir.path();

        let tree = PlatformsTree::load_search_path(&[root.to_path_buf()])
            .expect("an empty but EXISTING root is a valid tree");

        // Absent: the variant `or_builtin_rungs` keys its tolerant arm on.
        match tree.platform_param_rungs("threadx-linux") {
            Err(ConfigError::UnknownPlatform { name, .. }) => {
                assert_eq!(name, "threadx-linux");
            }
            other => panic!("expected UnknownPlatform, got {other:?}"),
        }

        // ...and with no platform rung, the ladder still answers with builtins
        // rather than dropping the knob.
        let resolved = tree.resolve_memory_from(
            "threadx-linux",
            &MemoryKnobs::default(),
            None,
            &|_| None,
            &[("heap_bytes", 4096)],
        );
        assert_eq!(resolved.len(), 1, "an absent rung must not drop the knob");
        assert_eq!(resolved[0].1.value, 4096);

        // Present but malformed: NOT UnknownPlatform, so it stays fatal.
        std::fs::create_dir_all(root.join("brokenplat")).expect("mkdir");
        std::fs::write(
            root.join("brokenplat/nros-platform.toml"),
            "[knobs.executor]\nmax_cbs = \"not a number\"\n",
        )
        .expect("write");
        let reloaded = PlatformsTree::load_search_path(&[root.to_path_buf()]);
        assert!(
            !matches!(reloaded, Err(ConfigError::UnknownPlatform { .. })),
            "a malformed descriptor must not be reported as an absent one — \
             the tolerant arm would then silently substitute builtins"
        );
    }

    use super::*;

    fn write_tree(files: &[(&str, &str)]) -> tempfile::TempDir {
        let tmp = tempfile::tempdir().unwrap();
        for (name, body) in files {
            let dir = tmp.path().join(name);
            fs::create_dir_all(&dir).unwrap();
            fs::write(dir.join(PLATFORM_CONFIG_FILENAME), body).unwrap();
        }
        tmp
    }

    fn no_env(_: &str) -> Option<String> {
        None
    }

    // ---- phase-400 W3 / RFC-0086 D2 — the transport tenant ----

    const CAPS_BOTH: &str = r#"
[capabilities]
ip_stack = true
serial = true
"#;

    #[test]
    fn transport_serial_implies_the_whole_dependent_set() {
        // The regression this tenant exists to prevent: fifteen hand-written
        // Kconfig lines per image, turning off links and drivers that a
        // transport choice already determines.
        let tmp = write_tree(&[("p", CAPS_BOTH)]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let env = |k: &str| (k == "NROS_TRANSPORT_KIND").then(|| "serial".to_string());
        let t = tree.resolve_transport("p", None, &env).unwrap();
        assert_eq!(t.kind.value, "serial");
        let off = [
            "links.tcp",
            "links.udp",
            "drivers.ethernet",
            "drivers.phy",
            "drivers.mdio",
            "net.ip_stack",
        ];
        for knob in off {
            let i = t.implied.iter().find(|i| i.knob == knob).expect(knob);
            assert!(!i.value, "{knob} should be implied off by serial");
            assert!(i.overridden_by.is_none());
        }
        let ser = t.implied.iter().find(|i| i.knob == "links.serial").unwrap();
        assert!(ser.value);
    }

    #[test]
    fn implies_is_imply_strength_not_select() {
        // Kconfig's `select` forces and can build an invalid config. Ours must
        // LOSE to a higher rung, and must record that it did.
        let tmp = write_tree(&[("p", CAPS_BOTH)]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let env = |k: &str| match k {
            "NROS_TRANSPORT_KIND" => Some("serial".to_string()),
            "NROS_ZENOH_LINK_TCP" => Some("1".to_string()),
            _ => None,
        };
        let t = tree.resolve_transport("p", None, &env).unwrap();
        let tcp = t.implied.iter().find(|i| i.knob == "links.tcp").unwrap();
        assert_eq!(tcp.overridden_by, Some(KnobSource::Env));
    }

    #[test]
    fn requires_is_hard_only_against_an_explicit_false() {
        // Absent is "nobody described this platform yet" and must not fail a
        // build that works; an explicit false is a described impossibility.
        let tmp = write_tree(&[
            ("undeclared", "[capabilities]\nserial = true\n"),
            ("denied", "[capabilities]\nip_stack = false\n"),
        ]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let tcp = |k: &str| (k == "NROS_TRANSPORT_KIND").then(|| "tcp".to_string());

        let t = tree.resolve_transport("undeclared", None, &tcp).unwrap();
        assert_eq!(t.kind.value, "tcp");
        assert_eq!(tree.transport_warnings("undeclared", &t).unwrap().len(), 1);

        assert!(matches!(
            tree.resolve_transport("denied", None, &tcp),
            Err(ConfigError::TransportCapabilityMissing { .. })
        ));
    }

    #[test]
    fn transport_kind_is_exactly_one_of() {
        // A typo must not silently select nothing: every implication would go
        // unapplied and the image would build with the wrong links on.
        let tmp = write_tree(&[("p", CAPS_BOTH)]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let env = |k: &str| (k == "NROS_TRANSPORT_KIND").then(|| "uart".to_string());
        assert!(matches!(
            tree.resolve_transport("p", None, &env),
            Err(ConfigError::UnknownTransport { .. })
        ));
    }

    // ---- phase-400 W6 — the executor sizing tenant ----

    #[test]
    fn executor_knobs_take_the_same_four_rung_ladder() {
        let tmp = write_tree(&[("p", "[knobs.executor]\nmax_cbs = 9\n")]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let defaults: &[(&str, usize)] = &[("max_cbs", 4), ("max_nodes", 4)];

        // platform beats builtin
        let r = tree.resolve_executor("p", None, &no_env, defaults).unwrap();
        let cbs = r.iter().find(|(n, _)| *n == "max_cbs").unwrap();
        assert_eq!(cbs.1.value, 9);
        assert_eq!(cbs.1.source, KnobSource::Platform);
        // an untouched knob still falls to the crate default
        let nodes = r.iter().find(|(n, _)| *n == "max_nodes").unwrap();
        assert_eq!(nodes.1.value, 4);
        assert_eq!(nodes.1.source, KnobSource::Builtin);

        // board beats platform
        let board = ExecutorKnobs {
            max_cbs: Some(11),
            ..Default::default()
        };
        let r = tree
            .resolve_executor("p", Some(&board), &no_env, defaults)
            .unwrap();
        assert_eq!(r.iter().find(|(n, _)| *n == "max_cbs").unwrap().1.value, 11);

        // env beats both, under the knob's EXISTING name
        let env = |k: &str| (k == "NROS_EXECUTOR_MAX_CBS").then(|| "13".to_string());
        let r = tree
            .resolve_executor("p", Some(&board), &env, defaults)
            .unwrap();
        let cbs = r.iter().find(|(n, _)| *n == "max_cbs").unwrap();
        assert_eq!(cbs.1.value, 13);
        assert_eq!(cbs.1.source, KnobSource::Env);
        assert_eq!(cbs.1.env_key, "NROS_EXECUTOR_MAX_CBS");
    }

    // ---- phase-400 W2 / RFC-0071 D8 — knobs keyed by backend ----

    #[test]
    fn knobs_accept_a_backend_nano_ros_has_never_heard_of() {
        let tmp = write_tree(&[("p", "[knobs.acme-rmw.tx]\nbatch = true\nflush_ms = 7\n")]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        // Parsing a third-party backend's knobs must not require nano-ros to
        // know its name — that is the whole point of de-keying the section.
        assert!(tree.resolve_tx("p", None, &no_env).is_ok());
    }

    #[test]
    fn a_mistyped_top_level_knob_key_is_reported_not_denied() {
        // The COST of de-keying `[knobs]`: serde cannot combine `flatten` with
        // `deny_unknown_fields`, so `[knobs.transprot]` no longer fails the
        // parse — it reads as a backend named "transprot". That is not a
        // regression we can close at parse time, because whether a key is a
        // valid backend name is only knowable once the rmw descriptors are
        // loaded, which serde cannot do. `unknown_backend_keys` is the
        // replacement diagnostic; this test pins both halves of the trade.
        let tmp = write_tree(&[("p", "[knobs.transprot]\ntx = {}\n")]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let knobs = tree.knobs_for("p").unwrap();
        assert!(knobs.backends.contains_key("transprot"));
        assert_eq!(
            knobs.unknown_backend_keys(&["zenoh", "cyclonedds"]),
            vec!["transprot".to_string()]
        );
        // and a real backend name is NOT reported
        assert!(knobs.unknown_backend_keys(&["transprot"]).is_empty());
    }

    #[test]
    fn legacy_zenoh_knobs_still_parse() {
        // The seven in-tree files must not change. Byte-identity of behaviour
        // is the acceptance rule phase-290 W2.c set for this schema.
        let tmp = write_tree(&[("p", "[knobs.zenoh.tx]\nbatch = true\n")]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let tx = tree.resolve_tx("p", None, &no_env).unwrap();
        assert!(tx.batch.value);
        assert_eq!(tx.batch.source, KnobSource::Platform);
    }

    /// phase-347 W6 — a SECOND `[build.<component>]` key parses.
    ///
    /// This is the whole point of the keying change and the only thing that
    /// could regress it: before, `BuildSection` was a struct with one `zenoh`
    /// field and `deny_unknown_fields`, so this input was a hard parse ERROR —
    /// a platform could describe exactly one backend's vendored C build.
    ///
    /// Asserts both keys survive independently, so a future component's block
    /// cannot silently overwrite or shadow zenoh's.
    /// phase-347 W6 — the seven REAL platform files still load, and each still
    /// exposes its zenoh block under the new keying. Behaviour-preserving is
    /// the claim; this is the check.
    #[test]
    fn real_config_tree_still_loads_zenoh_blocks() {
        let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .unwrap()
            .to_path_buf();
        let path = PlatformsTree::default_search_path(&repo, None);
        if !path.iter().any(|p| p.is_dir()) {
            return; // out-of-tree consumer; nothing to check
        }
        let tree =
            PlatformsTree::load_search_path(&path).expect("the real platform search path loads");
        assert!(
            !tree.files.is_empty(),
            "platform search path yielded no platform files"
        );
        let with_zenoh = tree
            .files
            .values()
            .filter(|f| f.build.contains_key("zenoh"))
            .count();
        assert!(
            with_zenoh >= 7,
            "expected >=7 platform files carrying a [build.zenoh] block, saw {with_zenoh}"
        );
    }

    /// issue 0534 — exactly which platforms keep cargo out of the vendored C
    /// build, asserted against the real tree.
    ///
    /// This is a TRIPWIRE, not a preference. `compiled_by = "platform"` is the
    /// difference between "the zpico resolver names this platform" and "a build
    /// script cc-compiles its `system/<platform>/*.c`", and on Zephyr the second
    /// cannot work: those sources need Zephyr's generated `version.h`. The
    /// property lived in a comment for one release and #529 walked straight
    /// through it, so it is checked in BOTH directions — a platform gaining the
    /// key and Zephyr losing it are equally a regression.
    #[test]
    fn only_zephyr_delegates_its_c_build_to_the_platform() {
        let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .unwrap()
            .to_path_buf();
        let root_path = PlatformsTree::default_search_path(&repo, None);
        let root = repo.join("config");
        if !root.is_dir() {
            return; // out-of-tree consumer; nothing to check
        }
        let tree =
            PlatformsTree::load_search_path(&root_path).expect("the real config/ tree loads");
        let manifest = tree.as_platform_manifest();
        let mut delegating: Vec<String> = Vec::new();
        let mut checked = 0usize;
        for name in tree.files.keys() {
            let Ok(resolved) = manifest.for_platform(name) else {
                continue;
            };
            checked += 1;
            if resolved.compiled_by == crate::manifest::CompiledBy::Platform {
                delegating.push(name.clone());
            }
        }
        assert!(
            checked >= 7,
            "resolved only {checked} platforms — this assertion would be vacuous"
        );
        delegating.sort();
        assert_eq!(
            delegating,
            vec!["zephyr".to_string()],
            "the set of platforms whose own build system compiles zenoh-pico changed. \
             Adding one is fine — say so here. Losing zephyr is issue 0534 returning: \
             cargo will cc-compile system/zephyr/*.c and fail on a missing version.h."
        );
    }

    /// phase-349 W1 — a platform answers to its aliases, and the directory
    /// name always works whether or not `names` is declared.
    #[test]
    fn an_alias_resolves_to_its_directory() {
        let tmp = write_tree(&[
            ("freertos", "names = [\"freertos\", \"freertos-lwip\"]\n"),
            ("posix", ""),
        ]);
        let tree = PlatformsTree::load(tmp.path()).expect("loads");

        assert_eq!(tree.resolve_alias("freertos-lwip"), "freertos");
        assert_eq!(tree.resolve_alias("freertos"), "freertos");
        // A file declaring no names still answers to its directory — that is
        // what every file meant before `names` existed.
        assert_eq!(tree.resolve_alias("posix"), "posix");
        // An unclaimed name comes back unchanged, so the caller's own
        // "unknown platform" error names what the USER asked for.
        assert_eq!(tree.resolve_alias("nope"), "nope");
    }

    /// The alias must work through the public lookups, not merely in
    /// `resolve_alias` — the point is that callers need no alias awareness.
    #[test]
    fn public_lookups_accept_an_alias() {
        let tmp = write_tree(&[(
            "freertos",
            "names = [\"freertos\", \"freertos-lwip\"]\n\
             [capabilities]\nthreads = true\n",
        )]);
        let tree = PlatformsTree::load(tmp.path()).expect("loads");

        let by_alias = tree.capabilities("freertos-lwip").expect("alias resolves");
        let by_dir = tree.capabilities("freertos").expect("dir resolves");
        assert_eq!(by_alias, by_dir);
        assert_eq!(by_alias.get("threads"), Some(&true));

        let no_env = |_: &str| None;
        assert!(tree.resolve_tx("freertos-lwip", None, &no_env).is_ok());
    }

    /// `inherits` names a sibling, and an alias there must resolve too —
    /// otherwise renaming a directory silently breaks its children.
    #[test]
    fn inherits_accepts_an_alias() {
        let tmp = write_tree(&[
            (
                "freertos",
                "names = [\"freertos\", \"freertos-lwip\"]\n\
                 [capabilities]\nthreads = true\n",
            ),
            ("child", "inherits = \"freertos-lwip\"\n"),
        ]);
        let tree = PlatformsTree::load(tmp.path()).expect("loads");
        let caps = tree.capabilities("child").expect("inherits via alias");
        assert_eq!(caps.get("threads"), Some(&true));
    }

    #[test]
    fn build_section_accepts_a_second_component() {
        let tmp = write_tree(&[(
            "zephyr",
            "[build.zenoh]\ndefines = [\"Z_ONE\"]\n\
             [build.xrce]\ndefines = [\"X_ONE\", \"X_TWO\"]\n",
        )]);
        let tree = PlatformsTree::load(tmp.path()).expect("a second [build.*] key must parse");
        let file = tree.files.get("zephyr").expect("zephyr file loaded");
        assert_eq!(
            file.build.get("zenoh").expect("zenoh block").defines,
            vec!["Z_ONE".to_string()],
        );
        assert_eq!(
            file.build.get("xrce").expect("xrce block").defines,
            vec!["X_ONE".to_string(), "X_TWO".to_string()],
        );
    }

    #[test]
    fn empty_tree_yields_builtins() {
        let tmp = write_tree(&[("zephyr", "")]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let tx = tree.resolve_tx("zephyr", None, &no_env).unwrap();
        assert!(!tx.batch.value);
        assert_eq!(tx.batch.source, KnobSource::Builtin);
        assert_eq!(tx.flush_ms.value, BUILTIN_TX_FLUSH_MS);
    }

    #[test]
    fn ladder_platform_board_env_order() {
        let tmp = write_tree(&[(
            "zephyr",
            "[capabilities]\nthreads = true\n[knobs.zenoh.tx]\nbatch = true\nflush_ms = 40\n",
        )]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();

        // platform rung
        let tx = tree.resolve_tx("zephyr", None, &no_env).unwrap();
        assert!(tx.batch.value);
        assert_eq!(tx.batch.source, KnobSource::Platform);
        assert_eq!(tx.flush_ms.value, 40);

        // board rung overrides platform
        let board = TxKnobs {
            batch: Some(false),
            split_lock: None,
            flush_ms: None,
        };
        let tx = tree.resolve_tx("zephyr", Some(&board), &no_env).unwrap();
        assert!(!tx.batch.value);
        assert_eq!(tx.batch.source, KnobSource::Board);
        assert_eq!(tx.flush_ms.value, 40); // untouched delta falls through

        // env rung overrides board — including explicit re-enable
        let env = |k: &str| (k == "ZPICO_TX_BATCH").then(|| "1".to_string());
        let tx = tree.resolve_tx("zephyr", Some(&board), &env).unwrap();
        assert!(tx.batch.value);
        assert_eq!(tx.batch.source, KnobSource::Env);
    }

    #[test]
    fn explicit_env_zero_overrides_on_default() {
        let tmp = write_tree(&[(
            "zephyr",
            "[knobs.zenoh.tx]\nbatch = true\nsplit_lock = true\n",
        )]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let env = |k: &str| (k == "ZPICO_TX_BATCH").then(|| "0".to_string());
        let tx = tree.resolve_tx("zephyr", None, &env).unwrap();
        assert!(!tx.batch.value, "set env 0 must beat an on-default");
        assert_eq!(tx.batch.source, KnobSource::Env);
        assert!(
            tx.split_lock.value,
            "untouched knob keeps the platform rung"
        );
    }

    #[test]
    fn inherits_chain_merges_child_wins() {
        let tmp = write_tree(&[
            (
                "generic",
                "[capabilities]\nthreads = true\n[knobs.zenoh.tx]\nflush_ms = 30\n",
            ),
            (
                "child",
                "inherits = \"generic\"\n[knobs.zenoh.tx]\nflush_ms = 60\n",
            ),
        ]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let tx = tree.resolve_tx("child", None, &no_env).unwrap();
        assert_eq!(tx.flush_ms.value, 60);
        let caps = tree.capabilities("child").unwrap();
        assert_eq!(caps.get("threads"), Some(&true));
    }

    #[test]
    fn unknown_knob_key_fails_loud() {
        let tmp = write_tree(&[("zephyr", "[knobs.zenoh.tx]\nbatchh = true\n")]);
        let err = PlatformsTree::load(tmp.path()).unwrap_err();
        assert!(format!("{err}").contains("batchh"), "{err}");
    }

    #[test]
    fn capability_check_downgrades_split_without_threads() {
        let tmp = write_tree(&[(
            "bare-metal",
            "[knobs.zenoh.tx]\nbatch = true\nsplit_lock = true\n",
        )]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let mut tx = tree.resolve_tx("bare-metal", None, &no_env).unwrap();
        let warnings = tree.capability_check("bare-metal", &mut tx).unwrap();
        assert_eq!(warnings.len(), 1);
        assert!(!tx.split_lock.value);
        assert!(tx.batch.value, "batch itself survives (spin-driven flush)");
    }

    #[test]
    fn arch_identical_duplicate_ok_conflict_errors() {
        let arch = "[arch.cortex-m3]\ntarget_match = \"thumbv7m\"\n";
        let tmp = write_tree(&[("a", arch), ("b", arch)]);
        assert!(PlatformsTree::load(tmp.path()).is_ok());

        let tmp = write_tree(&[
            ("a", arch),
            ("b", "[arch.cortex-m3]\ntarget_match = \"thumbv7em\"\n"),
        ]);
        assert!(matches!(
            PlatformsTree::load(tmp.path()),
            Err(ConfigError::ArchConflict { .. })
        ));
    }

    #[test]
    fn legacy_manifest_view_resolves_build_zenoh() {
        let tmp = write_tree(&[
            ("generic", "[build.zenoh]\ndefines = [\"A\"]\n"),
            (
                "child",
                "inherits = \"generic\"\n[build.zenoh]\ndefines = [\"B\"]\n",
            ),
        ]);
        let tree = PlatformsTree::load(tmp.path()).unwrap();
        let manifest = tree.as_platform_manifest();
        let resolved = manifest.for_platform("child").unwrap();
        assert_eq!(resolved.defines, vec!["A".to_string(), "B".to_string()]);
    }

    /// Issue 0979 — the regression, and it needs no env and no cwd fiddling.
    ///
    /// Cargo runs a test binary with cwd set to its own PACKAGE directory,
    /// exactly as it runs a build script. So this test executes from
    /// `packages/boards/nros-board-common`, where neither `packages/platform`
    /// nor `config` exists — which is precisely the state that made the old
    /// `current_dir()`-derived path resolve to nothing. A path with no
    /// existing root is a tree with no platforms in it.
    #[test]
    fn the_build_search_path_is_rooted_at_the_repo_not_the_calling_package() {
        let search = build_search_path();
        let existing: Vec<_> = search.iter().filter(|p| p.is_dir()).collect();
        assert!(
            !existing.is_empty(),
            "no root in the build search path exists, so the tree would be \
             empty and every platform lookup would fail: {search:?}"
        );
        assert!(
            existing.iter().any(|p| p.ends_with("packages/platform")),
            "the in-tree descriptor home must be reachable from a build \
             script's cwd: {search:?}"
        );
    }

    /// Issue 0979's other half: a wrong root must fail WHERE IT IS RESOLVED.
    ///
    /// Returning an empty tree is what turned this into `unknown platform
    /// \`posix\`` several frames later — a message about a platform that
    /// exists, whose only clue was a leading slash where the root should be.
    #[test]
    fn a_search_path_with_no_existing_root_is_an_error_naming_what_was_tried() {
        let missing = [PathBuf::from("/nonexistent-nros-0979/a")];
        let err = PlatformsTree::load_search_path(&missing)
            .expect_err("an all-missing search path must not yield an empty tree");
        assert!(
            matches!(err, ConfigError::NoSearchRoot { .. }),
            "expected NoSearchRoot, got {err:?}"
        );
        let msg = err.to_string();
        assert!(
            msg.contains("/nonexistent-nros-0979/a"),
            "the error must name every root it tried: {msg}"
        );
    }

    /// ...but SOME roots missing is still fine — that is what a search path is
    /// for, and tightening the all-missing case must not break a porter who
    /// prepends a tree of their own to the in-tree ones.
    #[test]
    fn a_search_path_keeps_skipping_individual_missing_roots() {
        let tmp = write_tree(&[("generic", "[build.zenoh]\ndefines = [\"A\"]\n")]);
        let tree = PlatformsTree::load_search_path(&[
            PathBuf::from("/nonexistent-nros-0979/b"),
            tmp.path().to_path_buf(),
        ])
        .expect("one existing root is enough");
        assert!(tree.files.contains_key("generic"));
    }
}
