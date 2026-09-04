//! The hosted edge of configuration: the process environment.
//!
//! issue 0687 / phase-359 W10. Every env var nano-ros honours is read HERE and
//! nowhere else. The core (`nros-node`) takes values — `ExecutorConfig::
//! resolve_with` accepts an [`EnvRung`](nros_node::EnvRung) of already-resolved fields — so it
//! needs no `std`, no `env` capability, and no stub on the five ports that have
//! no environment to read.
//!
//! That is the difference between this and the ABI ports W10 made for the
//! clock, sleep, tasks and the log sink: those name capabilities every RTOS
//! HAS. A process environment is not one. Modelling it as an ABI entry would
//! have put `return 0` in five ports and permanent surface in the header, for a
//! facility that exists on one platform family.
//!
//! What lives here:
//!
//! * [`ExecutorConfigEnvExt::from_env`] — the constructor that used to be an
//!   inherent method on `ExecutorConfig`. Bring the trait into scope (the
//!   prelude has it) and the call site spelling is unchanged.
//! * [`resolve_hosted`](crate::env::resolve_hosted) / [`try_resolve_hosted`](crate::env::try_resolve_hosted) — RFC-0045 precedence model A
//!   with the environment rung on top. These replace the old
//!   `ExecutorConfig::resolve(baked, hosted_env: bool)`, whose flag was a
//!   compile-time constant at every call site in the tree: `true` at exactly
//!   one board plus the two FFI entries, `false` everywhere else.
//! * [`rmw_selector`] — the one `$NROS_RMW` reader (issue 0687's first half).

use alloc::{
    boxed::Box,
    string::{String, ToString},
};
use nros_node::{BootConfig, BootConfigError, EnvRung, ExecutorConfig, RMW_SELECTOR_CAP};
use nros_rmw::SessionMode;

/// A frozen snapshot of the environment, and the backing store for the
/// `&'static str` fields a resolved [`ExecutorConfig`] hands out.
struct EnvCache {
    locator: String,
    mode: SessionMode,
    /// RFC-0045 model A — `NROS_NODE_NAME` env rung (issue #206 parity).
    node_name: String,
    /// issue 0687 — `$NROS_RMW`, through [`rmw_selector`] so the snapshot and
    /// the live reader cannot disagree about what "unset" means.
    rmw: Option<String>,
}

// `std::sync::OnceLock`, and it stays that way deliberately. The portable
// `nros_rmw::sync::Mutex` would cost a `spin` edge to remove a `std::` path
// from a module whose next line calls `std::env::var` — this whole file is
// compiled only under `env`, which REQUIRES `std` because a process
// environment is a `std` facility. Porting it would move the census number
// without changing what the build links, which is the definition of gaming the
// ratchet. (`metadata_mode`'s mutex was the opposite case and did move: there
// the lock was the ONLY std thing, so porting it made the capability
// `alloc`-only.)
static ENV_CACHE: std::sync::OnceLock<EnvCache> = std::sync::OnceLock::new();

/// The process-global env cache — and why tests do not share it.
///
/// Issue 0607 — resolution reads env PRESENCE live but env VALUES from here,
/// and a `OnceLock` freezes at whichever caller touched it first. Under
/// `cargo test` every unit test shares one process, so a later test's
/// `EnvGuard` moves what `std::env::var` reports while the cached VALUE stays
/// behind. The two then disagree:
///
/// ```text
/// assertion `left == right` failed
///   left: ""
///  right: "tcp/env:7447"
/// ```
///
/// It failed ~1 run in 5, never single-threaded, and a mutex cannot help: a
/// mutex serialises MUTATION, and this is a stale READ of a cache already
/// populated. Two tests had been weakened to tolerate it.
///
/// So tests rebuild per call — live env is the only coherent answer when the
/// env is what they are varying. Production keeps the `OnceLock`: nothing there
/// mutates the environment, so freezing it once is both correct and the point.
fn env_cache() -> &'static EnvCache {
    fn build() -> EnvCache {
        // Prefer NROS_LOCATOR / NROS_SESSION_MODE; accept legacy ZENOH_*
        // names with a deprecation warning.
        let locator = std::env::var("NROS_LOCATOR")
            .or_else(|_| {
                std::env::var("ZENOH_LOCATOR").inspect(|_| {
                    nros_log::log_warn!(
                        nros_log::get_logger("nros"),
                        "ZENOH_LOCATOR is deprecated; use NROS_LOCATOR instead"
                    );
                })
            })
            // Issue 0330 — unset env leaves the locator EMPTY (= absent); the
            // active backend substitutes its own default.
            .unwrap_or_default();
        let mode_str = std::env::var("NROS_SESSION_MODE")
            .or_else(|_| {
                std::env::var("ZENOH_MODE").inspect(|_| {
                    nros_log::log_warn!(
                        nros_log::get_logger("nros"),
                        "ZENOH_MODE is deprecated; use NROS_SESSION_MODE instead"
                    );
                })
            })
            .ok();
        let mode = match mode_str.as_deref() {
            Some("peer") => SessionMode::Peer,
            _ => SessionMode::Client,
        };
        let node_name = std::env::var("NROS_NODE_NAME").unwrap_or_default();
        EnvCache {
            locator,
            mode,
            node_name,
            rmw: rmw_selector().map(|s| s.as_str().to_string()),
        }
    }

    if cfg!(test) {
        Box::leak(Box::new(build()))
    } else {
        ENV_CACHE.get_or_init(build)
    }
}

/// **The** answer to "which RMW backend did the user select" — one reader, one
/// semantic, for every consumer.
///
/// phase-359 W10 / issue 0687. This variable had FOUR readers with THREE
/// semantics: `Executor::open` read it as raw OS bytes, `nros`'s
/// `open_session` as a UTF-8 string filtered for empty, `nros-c`'s entry as a
/// string passed through EVEN WHEN EMPTY, and `nros::init` as a string with an
/// `RMW_IMPLEMENTATION` fallback the other three did not have. "Which backend
/// did the user ask for" had four answers in one process.
///
/// Two decisions are baked in, and both are deliberate:
///
/// * **`$NROS_RMW` only.** `$RMW_IMPLEMENTATION` is NOT folded in, though it
///   was tempting and one reader did it. The two carry different vocabularies:
///   this selector is matched against the cffi registry's canonical names
///   (`zenoh`, `dds`, `cyclonedds`), while `RMW_IMPLEMENTATION` holds ROS names
///   (`rmw_cyclonedds_cpp`). Feeding a ROS name to `resolve_backend` yields
///   `Unknown` — an ERROR — where today it is ignored and the unique-backend
///   path runs. Unifying them without a mapping would convert "ignored" into
///   "fails to start". [`crate::init`](fn@crate::init) keeps its fallback for the `Context.rmw`
///   HINT, which is a different quantity.
/// * **Empty or non-UTF-8 means unset.** A name that is not UTF-8 cannot match
///   a registry entry, so treating it as absent is what the caller wants; the
///   old raw-bytes reader would have reported `Unknown` instead.
///
/// The return is `heapless::String` rather than `String` because
/// `RMW_SELECTOR_CAP` is a real bound, not a guess: it is the capacity of
/// `Executor::primary_rmw_name`, so a longer value cannot name a registry slot
/// and is reported as unset rather than truncated into a different backend's
/// name.
pub fn rmw_selector() -> Option<nros_core::heapless::String<RMW_SELECTOR_CAP>> {
    let raw = std::env::var_os("NROS_RMW")?;
    let s = raw.to_str()?;
    if s.is_empty() {
        return None;
    }
    nros_core::heapless::String::try_from(s).ok()
}

/// The environment as an [`EnvRung`], for [`ExecutorConfig::resolve_with`].
///
/// Presence is read LIVE (so a test that sets a var sees its effect) while
/// values come from the frozen [`env_cache`] — that split is what gives the
/// resolved config `&'static str` fields without leaking per call.
fn env_rung() -> Result<EnvRung<'static>, BootConfigError> {
    let cache = env_cache();

    let locator_present =
        std::env::var("NROS_LOCATOR").is_ok() || std::env::var("ZENOH_LOCATOR").is_ok();
    let domain_id = match std::env::var("ROS_DOMAIN_ID") {
        Ok(s) if !s.is_empty() => {
            // #206 — malformed is an ERROR, not a silent skip. The parse lives
            // here rather than in the core because only this side has the text.
            Some(
                s.trim()
                    .parse::<u32>()
                    .map_err(|_| BootConfigError::DomainIdParse)?,
            )
        }
        _ => None,
    };
    let node_name_present = std::env::var("NROS_NODE_NAME")
        .map(|s| !s.is_empty())
        .unwrap_or(false);

    Ok(EnvRung {
        locator: locator_present.then_some(cache.locator.as_str()),
        domain_id,
        // Session mode has no baked rung to fall through to, so the cache's
        // default (`Client`) is always the answer — it is stated rather than
        // conditional for exactly that reason.
        mode: Some(cache.mode),
        node_name: node_name_present.then_some(cache.node_name.as_str()),
        rmw: cache.rmw.as_deref(),
    })
}

/// RFC-0045 precedence model A with the environment on top:
/// `env (var set) > baked > compiled default`.
///
/// Fails loud on invalid identity input (repo rule: a bad domain id at boot is
/// a configuration error, never a silent domain-0 node). FFI shims that need a
/// return code call [`try_resolve_hosted`].
pub fn resolve_hosted<'a>(baked: BootConfig<'a>) -> ExecutorConfig<'a> {
    match try_resolve_hosted(baked) {
        Ok(cfg) => cfg,
        Err(e) => panic!("nros boot-config resolution failed: {e}"),
    }
}

/// Fallible [`resolve_hosted`] — returns [`BootConfigError`] instead of
/// panicking, so the C / C++ FFI shims can surface a return code.
///
/// - `ROS_DOMAIN_ID` set but non-numeric → `DomainIdParse` (the pre-#206 C++
///   header silently collapsed this to domain 0; the resolver silently ignored
///   it — both were wrong).
/// - any resolved domain id > `DOMAIN_ID_MAX` → `DomainIdRange`, INCLUDING a
///   baked one (the DDS backend would only fail later).
pub fn try_resolve_hosted<'a>(
    baked: BootConfig<'a>,
) -> Result<ExecutorConfig<'a>, BootConfigError> {
    ExecutorConfig::try_resolve_with(baked, Some(env_rung()?))
}

/// `ExecutorConfig::from_env()`, as an extension trait.
///
/// issue 0687 — this was an inherent method on `ExecutorConfig`, which is
/// defined in `nros-node`; an inherent method cannot be moved to another crate
/// and keep its spelling, and the spelling is what ~26 call sites (most of them
/// user-facing native examples) are written against. A trait keeps
/// `ExecutorConfig::from_env()` working wherever it is in scope — the
/// [`prelude`](crate::prelude) carries it, so a consumer that already writes
/// `use nros::prelude::*` changes nothing at all.
pub trait ExecutorConfigEnvExt {
    /// Create a configuration from environment variables.
    ///
    /// Reads:
    /// - `NROS_LOCATOR` — Middleware locator. Unset ⇒ empty (issue 0330: the
    ///   active RMW backend applies its own default; e.g. zenoh dials
    ///   `nros_rmw_zenoh::DEFAULT_LOCATOR`). Legacy name `ZENOH_LOCATOR` is
    ///   accepted with a deprecation warning.
    /// - `ROS_DOMAIN_ID` — ROS 2 domain ID (default: `0`).
    /// - `NROS_SESSION_MODE` — `"client"` or `"peer"` (default: `"client"`).
    ///   Legacy name `ZENOH_MODE` is accepted with a deprecation warning.
    /// - `NROS_RMW` — the backend selector, through [`rmw_selector`]. issue
    ///   0687: `Executor::open` used to read this itself; it now takes
    ///   `ExecutorConfig::rmw`, so a config built here still selects the
    ///   backend the user named.
    ///
    /// String values are cached in a process-global `OnceLock` on the first
    /// call and reused for the process lifetime — repeated calls do NOT
    /// accrete memory, and the returned `&'static str` fields point into that
    /// cache. Presence and the domain id are read live, so a caller that
    /// changes the environment between calls sees the change.
    ///
    /// **Panics** on a malformed or out-of-range `$ROS_DOMAIN_ID`, like
    /// [`resolve_hosted`] and the C / C++ entries. A boot identity that cannot
    /// be resolved is a configuration error; the alternative is a node silently
    /// running on domain 0, which is what issue #206 removed everywhere else.
    fn from_env() -> ExecutorConfig<'static>;
}

impl ExecutorConfigEnvExt for ExecutorConfig<'static> {
    fn from_env() -> ExecutorConfig<'static> {
        // issue 0687 — `from_env` IS `resolve_hosted` with nothing baked, and
        // saying so in code rather than in a test is the point. The two used to
        // be parallel implementations pinned together by an assertion
        // (`noop_resolve_matches_from_env`), and they had already drifted where
        // the assertion did not look: this one read `$ROS_DOMAIN_ID` through a
        // second, silent parse that turned a malformed or out-of-range value
        // into domain 0. Now a bad domain fails loud here exactly as it does
        // for `resolve`, `nros-c` and `nros-cpp` — the #206 rule, finally
        // uniform across all four.
        //
        // The one field that does NOT come from the rung is `node_name`:
        // `from_env` has never honoured `$NROS_NODE_NAME` (its doc lists what
        // it reads), and callers chain `.node_name(..)` immediately. Changing
        // that is a decision, not a cleanup.
        ExecutorConfig {
            node_name: "node",
            ..resolve_hosted(BootConfig::default())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Mutex, OnceLock};

    /// Process-wide mutex that serialises all env-touching tests.
    ///
    /// `cargo test` runs `#[test]`s in parallel within a single binary by
    /// default. Tests that mutate `NROS_LOCATOR` / `ROS_DOMAIN_ID` must hold
    /// this lock for the duration to avoid races with each other. (`cargo
    /// nextest` runs each test in its own process so the lock is always
    /// uncontended, but taking it is still correct.)
    fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        // Poison-TOLERANT, deliberately: three tests here assert that a bad
        // `$ROS_DOMAIN_ID` panics, and a panic while this guard is alive
        // poisons the mutex — which would then fail every LATER test in the
        // module for a reason that has nothing to do with what it asserts
        // (measured: 3 intended failures became 8). The data behind the lock is
        // `()`; there is no invariant a panic could have broken.
        LOCK.get_or_init(|| Mutex::new(()))
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    /// RAII guard that saves and restores a single env var.
    struct EnvGuard {
        key: &'static str,
        prev: Option<std::ffi::OsString>,
    }

    impl EnvGuard {
        fn set(key: &'static str, value: &str) -> Self {
            let prev = std::env::var_os(key);
            // SAFETY: serialised via env_lock().
            unsafe { std::env::set_var(key, value) };
            Self { key, prev }
        }

        fn unset(key: &'static str) -> Self {
            let prev = std::env::var_os(key);
            // SAFETY: serialised via env_lock().
            unsafe { std::env::remove_var(key) };
            Self { key, prev }
        }
    }

    impl Drop for EnvGuard {
        fn drop(&mut self) {
            // SAFETY: serialised via env_lock().
            unsafe {
                match &self.prev {
                    Some(v) => std::env::set_var(self.key, v),
                    None => std::env::remove_var(self.key),
                }
            }
        }
    }

    /// `resolve_hosted(BootConfig::default())` must be field-for-field
    /// identical to `from_env()`, regardless of what env vars are set.
    #[test]
    fn noop_resolve_matches_from_env() {
        let _g = env_lock();

        let resolved = resolve_hosted(BootConfig::default());
        let env_cfg = ExecutorConfig::from_env();

        assert_eq!(resolved.locator, env_cfg.locator);
        assert_eq!(resolved.mode, env_cfg.mode);
        assert_eq!(resolved.domain_id, env_cfg.domain_id);
        assert_eq!(resolved.node_name, env_cfg.node_name);
        assert_eq!(resolved.namespace, env_cfg.namespace);
        assert_eq!(resolved.rmw, env_cfg.rmw);
    }

    #[test]
    fn env_overrides_baked_on_hosted() {
        let _g = env_lock();
        let _e = EnvGuard::set("NROS_LOCATOR", "tcp/env:7447");

        let baked = BootConfig {
            locator: Some("tcp/baked:9999"),
            ..Default::default()
        };
        let resolved = resolve_hosted(baked);

        // Issue 0607 — the exact env string is observable: tests read live env
        // rather than a process-global cache that whichever test ran first had
        // already frozen. This was an `assert_ne!` against the baked value for
        // exactly that reason.
        assert_eq!(
            resolved.locator, "tcp/env:7447",
            "env locator must win over baked, with its own value"
        );
    }

    #[test]
    fn baked_used_when_env_unset_on_hosted() {
        let _g = env_lock();
        let _e1 = EnvGuard::unset("NROS_LOCATOR");
        let _e2 = EnvGuard::unset("ZENOH_LOCATOR");

        let baked = BootConfig {
            locator: Some("tcp/baked-only:8888"),
            ..Default::default()
        };
        let resolved = resolve_hosted(baked);

        assert_eq!(
            resolved.locator, "tcp/baked-only:8888",
            "baked locator must be used when env var is absent"
        );
    }

    #[test]
    fn per_field_independence_baked_name_env_locator() {
        let _g = env_lock();
        let _e = EnvGuard::set("NROS_LOCATOR", "tcp/env:7447");
        let _n = EnvGuard::unset("NROS_NODE_NAME");

        let baked = BootConfig {
            node_name: Some("my_talker"),
            ..Default::default()
        };
        let resolved = resolve_hosted(baked);

        assert_eq!(resolved.locator, "tcp/env:7447");
        assert_eq!(
            resolved.node_name, "my_talker",
            "baked node_name must apply even when locator comes from env"
        );
    }

    #[test]
    fn try_resolve_malformed_domain_env_errors() {
        let _l = env_lock();
        let _g = EnvGuard::set("ROS_DOMAIN_ID", "not-a-number");
        let err = match try_resolve_hosted(BootConfig::default()) {
            Err(e) => e,
            Ok(_) => panic!("expected DomainIdParse error"),
        };
        assert_eq!(err, BootConfigError::DomainIdParse);
    }

    #[test]
    fn try_resolve_domain_env_over_max_errors() {
        let _l = env_lock();
        let _g = EnvGuard::set("ROS_DOMAIN_ID", "233");
        let err = match try_resolve_hosted(BootConfig::default()) {
            Err(e) => e,
            Ok(_) => panic!("expected DomainIdRange error"),
        };
        assert_eq!(err, BootConfigError::DomainIdRange);
    }

    #[test]
    fn try_resolve_node_name_env_rung() {
        let _l = env_lock();
        let _g = EnvGuard::set("NROS_NODE_NAME", "env_node");
        let baked = BootConfig {
            node_name: Some("baked_node"),
            ..BootConfig::default()
        };
        let cfg = match try_resolve_hosted(baked) {
            Ok(c) => c,
            Err(e) => panic!("resolve failed: {e}"),
        };
        assert_eq!(
            cfg.node_name, "env_node",
            "env rung must override baked with its own value"
        );
    }

    /// issue 0687 — the selector reaches the config, which is how
    /// `Executor::open` still honours `$NROS_RMW` now that it does not read the
    /// environment itself. Both halves are asserted: a set value arrives, and
    /// an EMPTY value means unset rather than "a backend named the empty
    /// string" (which is what `nros-c` used to pass through).
    #[test]
    fn selector_reaches_the_config() {
        let _l = env_lock();

        let _g = EnvGuard::set("NROS_RMW", "cyclonedds");
        assert_eq!(rmw_selector().as_deref(), Some("cyclonedds"));
        assert_eq!(
            ExecutorConfig::from_env().rmw,
            Some("cyclonedds"),
            "from_env must carry the selector"
        );
        assert_eq!(
            resolve_hosted(BootConfig::default()).rmw,
            Some("cyclonedds")
        );

        let _g = EnvGuard::set("NROS_RMW", "");
        assert_eq!(rmw_selector(), None, "empty means unset");
        assert_eq!(ExecutorConfig::from_env().rmw, None);
    }

    /// issue 0687 — `from_env` and `resolve_hosted(default)` are the same
    /// resolution, and this asserts the fields the type does not force.
    #[test]
    fn from_env_is_resolve_hosted_with_nothing_baked() {
        let _l = env_lock();
        let _g = EnvGuard::set("NROS_LOCATOR", "tcp/agree:7447");
        let _d = EnvGuard::set("ROS_DOMAIN_ID", "42");

        let a = ExecutorConfig::from_env();
        let b = resolve_hosted(BootConfig::default());
        assert_eq!(a.locator, b.locator);
        assert_eq!(a.domain_id, b.domain_id);
        assert_eq!(a.mode, b.mode);
        assert_eq!(a.rmw, b.rmw);
        assert_eq!(a.namespace, b.namespace);
        // The one deliberate divergence: `from_env` does not take the node name
        // from the environment, and never has.
        assert_eq!(a.node_name, "node");
    }

    /// A malformed `$ROS_DOMAIN_ID` must fail LOUD here, as it does for
    /// `resolve_hosted` and the C / C++ entries. It used to resolve to domain 0
    /// silently — the #206 defect, surviving on this one path.
    #[test]
    #[should_panic(expected = "boot-config resolution failed")]
    fn from_env_rejects_a_malformed_domain() {
        let _l = env_lock();
        let _g = EnvGuard::set("ROS_DOMAIN_ID", "not-a-number");
        let _ = ExecutorConfig::from_env();
    }

    /// …and an out-of-range one, which the same path used to coerce to 0.
    #[test]
    #[should_panic(expected = "boot-config resolution failed")]
    fn from_env_rejects_an_out_of_range_domain() {
        let _l = env_lock();
        let _g = EnvGuard::set("ROS_DOMAIN_ID", "300");
        let _ = ExecutorConfig::from_env();
    }

    /// A selector longer than the executor's identity capacity cannot name a
    /// registry slot, so it is unset rather than truncated into some other
    /// backend's name.
    #[test]
    fn overlong_selector_is_unset_not_truncated() {
        let _l = env_lock();
        let long = "x".repeat(RMW_SELECTOR_CAP + 1);
        let _g = EnvGuard::set("NROS_RMW", &long);
        assert_eq!(rmw_selector(), None);
    }
}
