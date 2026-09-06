//! phase-291 (#211) — the canonical zephyr-leaf Kconfig→`rustc-env` bake.
//!
//! Every zephyr Rust leaf (standalone example or workspace `zephyr_entry`)
//! calls [`bake_nros_config`] from its `build.rs`, collapsing the previously
//! copy-pasted ~81-line file to:
//!
//! ```ignore
//! fn main() {
//!     zephyr_build::export_kconfig_bool_options(); // Kconfig→cfg bridge (phase-92.4)
//!     nros_zephyr_build::bake_nros_config();       // #17 locator/domain + 0163 XRCE bake
//! }
//! ```
//!
//! What it bakes (known-issue #17): `nros::main!`'s Zephyr branch and
//! `nros::zephyr_component_main!` read `option_env!("NROS_LOCATOR")` /
//! `option_env!("NROS_DOMAIN_ID")` at compile time; without a baked value the
//! locator falls back to EMPTY → zenoh-pico multicast scouting, which
//! native_sim NSOS can never satisfy (no `connect()` is ever issued). The C
//! API path consumes `CONFIG_NROS_ZENOH_LOCATOR` from Kconfig directly; this
//! helper re-exports the same Kconfig values so Kconfig stays the single
//! source of truth for BOTH languages. (`DOTCONFIG` — the generated
//! `.config` path — is exported by the Zephyr build system.)
//!
//! The bake MUST run in the LEAF's own `build.rs`: `cargo:rustc-env` from a
//! dependency's build script never reaches other crates' compilation, and the
//! `option_env!` reads expand in the leaf. That is why this is a shared
//! build-DEPENDENCY, not logic inside a runtime crate.
//!
//! Zero dependencies by design: upstream `zephyr-build` resolves as a
//! west-module PATH dep only (a leaf `Cargo.lock` entry with no `source =`),
//! so depending on it here would break host `cargo check --workspace`. The
//! `export_kconfig_bool_options()` call therefore stays in the leaf.

use std::{env, fs};

/// Bake the nros Kconfig values into `rustc-env` directives:
///
/// - `CONFIG_NROS_ZENOH_LOCATOR` → `NROS_LOCATOR` (quoted string, phase-225)
/// - `CONFIG_NROS_DOMAIN_ID` → `NROS_DOMAIN_ID` (integer, issue 0161)
/// - issue 0163 — when `CONFIG_NROS_RMW_XRCE=y`, synthesize the `host:port`
///   agent locator from `CONFIG_NROS_XRCE_AGENT_{ADDR,PORT}` (defaults
///   `127.0.0.1:2018`) into the SAME `NROS_LOCATOR` env (mutually exclusive
///   with the zenoh bake — an image selects exactly one RMW). Self-gated, so
///   zenoh-only images (and workspace entries) are unaffected.
///
/// No-op (beyond `rerun-if-env-changed`) when `DOTCONFIG` is unset or the
/// Kconfigs are absent/empty — a host `cargo check` of a leaf stays quiet.
pub fn bake_nros_config() {
    println!("cargo:rerun-if-env-changed=DOTCONFIG");
    println!("cargo:rerun-if-env-changed=NROS_LOCATOR");
    println!("cargo:rerun-if-env-changed=NROS_DOMAIN_ID");
    let Some(body) = env::var("DOTCONFIG").ok().and_then(|p| {
        println!("cargo:rerun-if-changed={p}");
        fs::read_to_string(&p).ok()
    }) else {
        return;
    };
    for line in bake_directives(&body) {
        println!("{line}");
    }
}

/// Pure core of [`bake_nros_config`]: `.config` body → the `cargo:rustc-env`
/// directive lines. Split out so tests assert emission without a cargo run.
fn bake_directives(dotconfig: &str) -> Vec<String> {
    let mut out = Vec::new();
    if let Some(val) = kconfig_str(dotconfig, "CONFIG_NROS_ZENOH_LOCATOR") {
        out.push(format!("cargo:rustc-env=NROS_LOCATOR={val}"));
    }
    if let Some(val) = kconfig_raw(dotconfig, "CONFIG_NROS_DOMAIN_ID") {
        out.push(format!("cargo:rustc-env=NROS_DOMAIN_ID={val}"));
    }
    // Issue 0163 — the XRCE path has no CONFIG_NROS_ZENOH_LOCATOR; its agent
    // endpoint lives in CONFIG_NROS_XRCE_AGENT_{ADDR,PORT}. Synthesize the
    // `host:port` locator the xrce session parser expects.
    if kconfig_raw(dotconfig, "CONFIG_NROS_RMW_XRCE").as_deref() == Some("y") {
        let addr = kconfig_str(dotconfig, "CONFIG_NROS_XRCE_AGENT_ADDR")
            .unwrap_or_else(|| "127.0.0.1".to_string());
        let port = kconfig_raw(dotconfig, "CONFIG_NROS_XRCE_AGENT_PORT")
            .unwrap_or_else(|| "2018".to_string());
        out.push(format!("cargo:rustc-env=NROS_LOCATOR={addr}:{port}"));
    }
    out
}

/// Resolve a build-script tuning knob: explicit env var, else the Zephyr
/// `.config` named by `$DOTCONFIG`, else `default`.
///
/// # Why a build script has to read `.config` at all (issue 0460)
///
/// `zephyr/cmake/nros_cargo_build.cmake` exports every resolved knob with
/// `set(ENV{...})`, which only touches the CONFIGURE-time cmake process. The C
/// lane survives that because `nros_cargo_build()` re-bakes the vars into its
/// build command (`cmake -E env …`). The RUST lane's command is built by
/// zephyr-lang-rust's `rust_cargo_application`, which passes its own fixed
/// variable list and inherits nothing — so **every Zephyr Rust image compiled
/// its crates' DEFAULTS whatever Kconfig said**. Measured on an image whose
/// `.config` said `CONFIG_NROS_EXECUTOR_MAX_CBS=16`: zero occurrences in
/// `build.ninja`, and the crate compiled 4.
///
/// `DOTCONFIG` *is* in that command's environment, so the value is read from
/// the file rather than by teaching the vendored module a new variable.
///
/// # Why `kconfig_key` is a separate argument
///
/// The env name and the Kconfig name are NOT the same string for every knob:
/// `nros-node` reads `NROS_EXECUTOR_MAX_CBS` ↔ `CONFIG_NROS_EXECUTOR_MAX_CBS`
/// (mechanical), but the zenoh shim reads `ZPICO_MAX_QUERYABLES` ↔
/// `CONFIG_NROS_MAX_QUERYABLES` (not). The pairing is declared by
/// `_nros_resolve_knob()` in `nros_cargo_build.cmake`, so a caller here passes
/// the same pair; `check-kconfig-knob-forwarding` holds the two lists together.
///
/// Explicit env still wins, so the C lane and any manual override are
/// unchanged.
pub fn knob_usize(env_name: &str, kconfig_key: &str, default: usize) -> usize {
    println!("cargo:rerun-if-env-changed={env_name}");
    if let Some(v) = env::var(env_name).ok().and_then(|v| v.parse().ok()) {
        return v;
    }
    match dotconfig(kconfig_key) {
        KnobSource::Value(v) => v,
        // The key is genuinely absent from a `.config` we READ: a Kconfig int
        // left at its default is not written to the file, so this is the
        // normal case and the crate default is the right answer.
        KnobSource::AbsentFromConfig => default,
        // Issue 1134 — `DOTCONFIG` names a file we cannot read. This used to be
        // `.unwrap_or(default)`, indistinguishable from the arm above, and that
        // conflation is the bug: every knob silently takes its crate default
        // and the image is built to sizes its configuration never asked for. A
        // 64 KiB platform heap under a 448 KiB executor arena reads as a
        // runtime fault, not a build one, which is why it cost a downstream
        // consumer a workaround instead of a bug report.
        KnobSource::ConfigUnreadable(path, why) => panic!(
            "nros-zephyr-build: DOTCONFIG={path} could not be read ({why}), so \
             `{kconfig_key}` cannot be resolved.\n  \
             Refusing to fall back to the crate default ({default}): a Zephyr \
             image built from crate defaults compiles, links, and then behaves \
             as though its Kconfig said nothing (issue 1134).\n  \
             This usually means a RECONFIGURE ran without the environment the \
             original configure had — `west build -t run` re-enters the build \
             graph, and the run inherits whatever the shell provides."
        ),
    }
}

/// Where a knob's value came from — or why it did not.
///
/// Issue 1134. These three outcomes were one `Option`, and two of them meant
/// opposite things: "the config says nothing, so use the default" is correct,
/// while "there is a config and we could not read it" is a build that must
/// stop. Collapsing them is what let a reconfigure silently rebuild an image
/// to crate defaults.
#[derive(Debug)]
pub enum KnobSource {
    /// Read from the `.config`.
    Value(usize),
    /// No `DOTCONFIG` (not a Zephyr build at all), or the key is absent from a
    /// `.config` that WAS read.
    AbsentFromConfig,
    /// `DOTCONFIG` names a file, and reading it failed.
    ConfigUnreadable(String, String),
}

/// [`dotconfig_usize`], keeping WHY it produced no value.
pub fn dotconfig(kconfig_key: &str) -> KnobSource {
    println!("cargo:rerun-if-env-changed=DOTCONFIG");
    let Ok(path) = env::var("DOTCONFIG") else {
        // Not a Zephyr build. Deliberately not an error: these same build
        // scripts compile for the host and for every other platform.
        return KnobSource::AbsentFromConfig;
    };
    println!("cargo:rerun-if-changed={path}");
    match fs::read_to_string(&path) {
        Ok(body) => match kconfig_usize(&body, kconfig_key) {
            Some(v) => KnobSource::Value(v),
            None => KnobSource::AbsentFromConfig,
        },
        Err(e) => KnobSource::ConfigUnreadable(path, e.to_string()),
    }
}

/// Read `<kconfig_key>=<int>` out of the `.config` named by `$DOTCONFIG`.
///
/// Kconfig ints are unquoted; anything else (missing file, missing key,
/// non-numeric) yields `None` so the caller falls through to its default.
pub fn dotconfig_usize(kconfig_key: &str) -> Option<usize> {
    match dotconfig(kconfig_key) {
        KnobSource::Value(v) => Some(v),
        // NOTE the flattening: this view cannot tell "absent" from
        // "unreadable", so neither can its callers. That is issue 1134 in one
        // line, and it is why `knob_usize` matches on [`dotconfig`] instead.
        _ => None,
    }
}

/// Pure core of [`dotconfig_usize`]: `.config` body → the key's integer value.
///
/// A bool option reads `y` (an unset bool is absent from the file, never `n`),
/// and the cmake side resolves those to `1`/`0` for the same knob — the C shim
/// takes `-DZPICO_TX_BATCH=1`. Map it the same way here so a tri-state knob
/// does not silently fall through to the crate default on the Rust lane.
fn kconfig_usize(body: &str, key: &str) -> Option<usize> {
    match kconfig_raw(body, key)?.as_str() {
        "y" => Some(1),
        "n" => Some(0),
        v => v.parse().ok(),
    }
}

/// `CONFIG_X="value"` → `Some("value")`; unset/empty → `None`.
fn kconfig_str(body: &str, key: &str) -> Option<String> {
    let raw = kconfig_raw(body, key)?;
    let val = raw.trim_matches('"');
    (!val.is_empty()).then(|| val.to_string())
}

/// `CONFIG_X=rhs` → `Some(rhs)` (verbatim, trimmed); unset/empty → `None`.
fn kconfig_raw(body: &str, key: &str) -> Option<String> {
    let prefix = format!("{key}=");
    body.lines()
        .find_map(|l| l.strip_prefix(&prefix))
        .map(str::trim)
        .filter(|v| !v.is_empty())
        .map(str::to_string)
}

#[cfg(test)]
mod tests {
    /// Issue 1134 — the three outcomes must stay three.
    ///
    /// These exercise [`dotconfig`] through the real `DOTCONFIG` environment
    /// variable, which is process-global, so they live in ONE test rather than
    /// three: `cargo test` runs them on threads and a second test mutating the
    /// same variable would make both flaky. The repo has been bitten by exactly
    /// that shape before (`nros_tests::unique_ros_domain_id`).
    #[test]
    fn a_knob_distinguishes_absent_from_unreadable() {
        use std::io::Write;

        // SAFETY: `set_var`/`remove_var` are unsound only with concurrent
        // readers of the environment; this test owns the variable for its
        // duration and no other test in this crate touches it.
        let restore = env::var("DOTCONFIG").ok();

        // 1. No DOTCONFIG at all — not a Zephyr build. NOT an error: the same
        //    build scripts compile for the host.
        unsafe { env::remove_var("DOTCONFIG") };
        assert!(
            matches!(dotconfig("CONFIG_ANY"), KnobSource::AbsentFromConfig),
            "no DOTCONFIG must be AbsentFromConfig, not an error"
        );

        // 2. A readable .config that does not mention the key. Correct to take
        //    the crate default: Kconfig does not write an int left at default.
        let dir = std::env::temp_dir().join(format!("nros-1134-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let cfg = dir.join(".config");
        let mut f = std::fs::File::create(&cfg).unwrap();
        writeln!(f, "CONFIG_SOMETHING_ELSE=7").unwrap();
        drop(f);
        unsafe { env::set_var("DOTCONFIG", &cfg) };
        assert!(
            matches!(dotconfig("CONFIG_ABSENT_KEY"), KnobSource::AbsentFromConfig),
            "a key absent from a READ config must be AbsentFromConfig"
        );
        assert!(
            matches!(dotconfig("CONFIG_SOMETHING_ELSE"), KnobSource::Value(7)),
            "a key present in a read config must yield its value"
        );

        // 3. DOTCONFIG names a file that is not there. THIS is the one that
        //    used to be indistinguishable from case 2, and taking the crate
        //    default here is how an image gets built to sizes its Kconfig never
        //    asked for.
        unsafe { env::set_var("DOTCONFIG", dir.join("does-not-exist")) };
        let got = dotconfig("CONFIG_ANY");
        assert!(
            matches!(got, KnobSource::ConfigUnreadable(..)),
            "an unreadable DOTCONFIG must be ConfigUnreadable, got {got:?}"
        );

        match restore {
            Some(v) => unsafe { env::set_var("DOTCONFIG", v) },
            None => unsafe { env::remove_var("DOTCONFIG") },
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    use super::*;

    /// The tree's `-1` DERIVE sentinel reads as NO VALUE here.
    ///
    /// Every `-1 = derive` Kconfig knob (`NROS_EXECUTOR_MAX_CBS`,
    /// `NROS_SUBSCRIPTION_BUFFER_SIZE`, `NROS_EXECUTOR_BACKING_U64S`, …) reaches
    /// its reading build script through this function, and each depends on the
    /// sentinel falling through to the crate's own default rather than being
    /// taken as a size. Nothing said so, because the behaviour comes from
    /// `"-1".parse::<usize>()` failing rather than from a branch anyone wrote —
    /// so a future reader "fixing" this to parse an `i64` would silently hand
    /// every one of those knobs a value of `-1 as usize`. Issues 0940 / 1171.
    #[test]
    fn the_derive_sentinel_is_not_a_size() {
        let cfg = "CONFIG_NROS_EXECUTOR_BACKING_U64S=-1\n\
                   CONFIG_NROS_EXECUTOR_MAX_CBS=-1\n\
                   CONFIG_NROS_EXECUTOR_MAX_SC=8\n";
        assert_eq!(
            kconfig_usize(cfg, "CONFIG_NROS_EXECUTOR_BACKING_U64S"),
            None
        );
        assert_eq!(kconfig_usize(cfg, "CONFIG_NROS_EXECUTOR_MAX_CBS"), None);
        // A stated size still arrives — the sentinel must not swallow the
        // legitimate values beside it.
        assert_eq!(kconfig_usize(cfg, "CONFIG_NROS_EXECUTOR_MAX_SC"), Some(8));
        // `0` is a DIFFERENT answer from the sentinel and must survive: it is
        // "decline the static" for the backing knob (phase-392 W6) and "this
        // image's types all fit the small class" for the payload trio.
        assert_eq!(
            kconfig_usize(
                "CONFIG_NROS_EXECUTOR_BACKING_U64S=0\n",
                "CONFIG_NROS_EXECUTOR_BACKING_U64S"
            ),
            Some(0)
        );
    }

    #[test]
    fn zenoh_locator_and_domain_bake() {
        let cfg = "CONFIG_NROS_RMW_ZENOH=y\n\
                   CONFIG_NROS_ZENOH_LOCATOR=\"tcp/127.0.0.1:7456\"\n\
                   CONFIG_NROS_DOMAIN_ID=42\n";
        assert_eq!(
            bake_directives(cfg),
            vec![
                "cargo:rustc-env=NROS_LOCATOR=tcp/127.0.0.1:7456".to_string(),
                "cargo:rustc-env=NROS_DOMAIN_ID=42".to_string(),
            ]
        );
    }

    #[test]
    fn unset_and_empty_are_no_ops() {
        assert!(bake_directives("").is_empty());
        assert!(bake_directives("CONFIG_NROS_ZENOH_LOCATOR=\"\"\n").is_empty());
        // A different key sharing the prefix must not match.
        assert!(bake_directives("CONFIG_NROS_ZENOH_LOCATOR_EXTRA=\"x\"\n").is_empty());
    }

    #[test]
    fn xrce_synthesis_with_explicit_endpoint() {
        let cfg = "CONFIG_NROS_RMW_XRCE=y\n\
                   CONFIG_NROS_XRCE_AGENT_ADDR=\"192.0.2.7\"\n\
                   CONFIG_NROS_XRCE_AGENT_PORT=8888\n";
        assert_eq!(
            bake_directives(cfg),
            vec!["cargo:rustc-env=NROS_LOCATOR=192.0.2.7:8888".to_string()]
        );
    }

    #[test]
    fn xrce_synthesis_defaults() {
        assert_eq!(
            bake_directives("CONFIG_NROS_RMW_XRCE=y\n"),
            vec!["cargo:rustc-env=NROS_LOCATOR=127.0.0.1:2018".to_string()]
        );
    }

    #[test]
    fn xrce_absent_emits_nothing() {
        // `# CONFIG_NROS_RMW_XRCE is not set` — the Kconfig-disabled shape.
        let cfg = "# CONFIG_NROS_RMW_XRCE is not set\n\
                   CONFIG_NROS_XRCE_AGENT_ADDR=\"10.0.0.1\"\n";
        assert!(bake_directives(cfg).is_empty());
    }
}
