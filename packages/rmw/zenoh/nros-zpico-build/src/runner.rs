//! Build script for zpico-sys
//!
//! This builds:
//! 1. zenoh-pico C library (via CMake for native, sources for embedded)
//! 2. The zpico C layer (zpico.c)
//! 3. Generates C header from Rust FFI declarations (cbindgen)

use std::{
    env,
    path::{Path, PathBuf},
};

// Phase 149.5 — shared manifest/policy parser moved to
// `nros-board-common`. Re-import as local module aliases so every
// `manifest::*` / `policy::*` reference below stays unchanged.
use nros_board_common::{manifest, platform_config, policy};

use policy::{LinkFeatures, LinkPolicy};

use crate::{
    add_zenoh_pico_core_sources, apply_arch, arch_matches, detect_riscv_compiler,
    get_picolibc_sysroot, has_picolibc_specs, is_embedded_target, read_symbol_size,
};

type ShimConfig = crate::ShimConfig;
type ZenohBufferConfig = crate::ZenohBufferConfig;

/// Queryables the ROS parameter services claim, mirroring
/// `nros_node::parameter_services::PARAM_SERVICE_QUERYABLES`.
///
/// A MIRROR, and the only one left. This crate is a build-script helper: it
/// cannot depend on `nros-node`, so it cannot read the constant, and cargo does
/// not expose another crate's features to a build script either. Held to the
/// definition by `check-infra-queryable-counts`, which is why the seven prose
/// spellings this replaced could drift and this one cannot. See phase-392 W5.
const PARAM_SERVICE_QUERYABLES: usize = 6;

/// Queryables the REP-2002 lifecycle services claim, mirroring
/// `nros_node::lifecycle_services::LIFECYCLE_SERVICE_QUERYABLES`. FIVE, not six.
const LIFECYCLE_SERVICE_QUERYABLES: usize = 5;

/// Headroom for a build that declares nothing.
///
/// This is the pre-phase-392-W5 embedded budget, kept EXACTLY so an undeclared
/// embedded image sizes as it always did. W5.f retires it together with the
/// hosted guess once every image declares; until then it is the fallback, not
/// the answer.
const UNDECLARED_HEADROOM: usize = 8;

/// RTOS `target_os` values that are NOT hosted, however much they name
/// themselves (issue 1028).
///
/// These targets have a `target_os` and even a `target_family = "unix"`, so the
/// obvious test — `target_os != "none"` — reads them as Linux-class hosts.
///
/// This list is the SSoT for the question and `check-rtos-target-os` holds two
/// other answers to it: every triple this tree names must resolve to a
/// `target_os` accounted for HERE, and every `cfg` predicate asking the hosted
/// question must name the reachable subset (a `cfg` cannot call a function, so
/// it has to re-spell the answer — `nros-macros` and `nros-rmw-zenoh`'s
/// `effective_client_locator` both do).
///
/// The third answer in this crate is [`crate::is_embedded_target`], which takes
/// the TRIPLE rather than the `target_os` and already names `nuttx`. It is a
/// different input for a different decision (which C toolchain and sysroot to
/// use), not a duplicate — but if you are about to write a fourth, it is one of
/// these two.
const RTOS_TARGET_OS: &[&str] = &["nuttx", "espidf", "horizon", "vita", "psp"];

/// "Is this a hosted target?" and "does `target_os` have a value?" are
/// different questions, and NuttX is the counterexample that proves it.
///
/// issue 1028 — `armv7a-nuttx-eabihf` reports `target_os = "nuttx"` and
/// `target_family = "unix"`, so `target_os != "none"` classified an RTOS as
/// hosted and handed it the 32-slot queryable budget written for Linux.
/// MEASURED on `examples/qemu-arm-nuttx/cpp/action-client`, an image that opens
/// ZERO queryables: `SERVICE_BUFFERS` was 142,336 B of `.bss` (32 x 4,448)
/// against 35,584 B at the embedded budget. Harmless at qemu-virt's 126 MB;
/// not harmless on a real part.
///
/// `target_os = "none"` still means bare-metal, so it stays unhosted; this only
/// adds the RTOSes that DO name themselves.
fn target_os_is_hosted(target_os: Option<&str>) -> bool {
    match target_os {
        None | Some("none") => false,
        Some(os) => !RTOS_TARGET_OS.contains(&os),
    }
}

#[cfg(test)]
mod hosted_tests {
    use super::*;

    #[test]
    fn an_rtos_that_names_itself_is_not_hosted() {
        // issue 1028's exact regression: this returned `true` and cost 106,752 B.
        assert!(!target_os_is_hosted(Some("nuttx")));
        assert!(!target_os_is_hosted(Some("espidf")));
    }

    #[test]
    fn bare_metal_and_absent_stay_unhosted() {
        assert!(!target_os_is_hosted(Some("none")));
        assert!(!target_os_is_hosted(None));
    }

    #[test]
    fn real_hosts_are_still_hosted() {
        // The arm this predicate exists to serve must keep working, or the fix
        // trades one wrong budget for another.
        assert!(target_os_is_hosted(Some("linux")));
        assert!(target_os_is_hosted(Some("macos")));
        assert!(target_os_is_hosted(Some("windows")));
    }
}

/// The queryable-table default, from the declaration when there is one.
///
/// phase-392 W5.d — `SERVICE_BUFFERS` is `ZPICO_MAX_SESSIONS *
/// ZPICO_MAX_QUERYABLES` service buffers, so this number is 4,504 bytes of
/// static RAM per slot. It used to be `if hosted { 32 } else { 8 }`: a literal
/// picked for headroom because nothing here knew the answer, costing a native
/// talker 144,128 bytes for services it does not have.
///
/// `NROS_DECLARED_SERVICE_SERVERS` is the application's own count, resolved
/// from the SystemModel and delivered by the same path phase-351 W5 uses for
/// board facts (`corrosion_set_env_vars`, which reaches cargo where
/// `set(ENV{...})` does not — issue 0460). The infrastructure counts are added
/// HERE rather than by whoever computes that figure, because codegen sees the
/// user's entities and never the runtime's: deriving the total from the app's
/// service count alone is issue 0460's defect exactly.
///
/// `NROS_DECLARED_INFRA_QUERYABLES` carries whether those runtime services are
/// compiled in. Absent, both are assumed present — the safe direction, since
/// over-reserving wastes RAM while under-reserving fails at boot.
///
/// With no declaration at all this returns the historical embedded budget and
/// says nothing; W5.e turns that case into a build-time failure, which needs
/// the hand-written-`main` question settled first (phase-392 W5, Open).
fn resolve_queryable_default() -> QueryableSizing {
    // WATCH what we READ. Both were consumed here and neither was declared, so
    // cargo had no reason to re-run this script when a declaration changed: an
    // entry that gained or lost a service server kept its previously-sized
    // tables until something ELSE forced a rebuild. The sizing then reads as
    // applied while being stale — the shape issue 0491 documents for path
    // variables, here with no watch at all.
    //
    // It also makes the knob untestable by hand: setting either variable and
    // rebuilding produces a byte-identical image, which is exactly how this was
    // found.
    println!("cargo:rerun-if-env-changed=NROS_DECLARED_SERVICE_SERVERS");
    println!("cargo:rerun-if-env-changed=NROS_DECLARED_INFRA_QUERYABLES");
    let declared = std::env::var("NROS_DECLARED_SERVICE_SERVERS").ok();
    let infra = std::env::var("NROS_DECLARED_INFRA_QUERYABLES").ok();
    QueryableSizing {
        default: queryable_default_from(
            declared.as_deref(),
            infra.as_deref(),
            target_os_is_hosted(std::env::var("CARGO_CFG_TARGET_OS").as_deref().ok()),
        ),
        floor: queryable_floor_from(declared.as_deref(), infra.as_deref()),
        declared: declared.is_some() || infra.is_some(),
    }
}

/// What the queryable table was sized from, and what it may not go below.
struct QueryableSizing {
    /// The size to use when nothing overrides it.
    default: usize,
    /// The DERIVED lower bound: slots the image provably needs before the
    /// application declares anything. See [`queryable_floor_from`].
    floor: usize,
    /// Whether any part of this came from a declaration.
    declared: bool,
}

/// The slots this image PROVABLY needs, as opposed to the ones it is budgeted.
///
/// phase-392 W5.f — `ZPICO_MAX_QUERYABLES` stops being the primary input and
/// becomes an override, and an override has to be checked rather than trusted.
/// But the check must compare against something DERIVED: [`queryable_default_from`]
/// adds [`UNDECLARED_HEADROOM`] whenever the application count is unavailable,
/// and refusing a build for being under a guess would be the same defect one
/// level up.
///
/// The infrastructure cost is not a guess. An image compiled with the ROS
/// parameter services claims [`PARAM_SERVICE_QUERYABLES`] slots at boot, with
/// the REP-2002 lifecycle services [`LIFECYCLE_SERVICE_QUERYABLES`], and a
/// declared application service server claims one each. Below that sum the
/// image cannot start, which is issue 0460 exactly: an 8-slot table against
/// eleven infrastructure queryables, discovered at boot as a bare
/// `ServiceServerCreationFailed`. That is now a BUILD failure, where the
/// person who set the knob is standing.
///
/// Undeclared images have a floor of zero: nothing is known, so nothing is
/// provable, and the historical budgets apply unchecked.
fn queryable_floor_from(declared: Option<&str>, infra: Option<&str>) -> usize {
    if declared.is_none() && infra.is_none() {
        return 0;
    }
    let app = declared
        .and_then(|v| v.trim().parse::<usize>().ok())
        .unwrap_or(0);
    app + infra_queryables(infra)
}

/// phase-392 W5.f — `ZPICO_MAX_QUERYABLES` as a CHECKED override.
///
/// Two mechanisms deciding one number is how this phase's other defects were
/// born, so the knob stops being an independent opinion: below the derived
/// floor it is refused, and below the budgeted default it is reported.
///
/// The two levels are not the same claim and must not be conflated. The FLOOR
/// is provable — an image carrying the parameter and lifecycle services claims
/// eleven slots before the application declares anything, so a smaller table
/// cannot boot. The DEFAULT adds [`UNDECLARED_HEADROOM`] for an application
/// count nothing can supply yet; being under THAT is a plausible problem, not a
/// certain one, and refusing a build over a guess is the defect this wave
/// exists to remove.
///
/// This is issue 0460 caught one stage earlier. `CONFIG_NROS_MAX_QUERYABLES`
/// defaults to 8 in `zephyr/Kconfig`, and an entry enabling both service
/// families needs eleven: that image used to build cleanly and die at boot with
/// a bare `ServiceServerCreationFailed`.
fn check_queryable_override(requested: usize, sizing: &QueryableSizing) {
    if !sizing.declared {
        return;
    }
    if requested < sizing.floor {
        panic!(
            "ZPICO_MAX_QUERYABLES={requested} is below this image's DERIVED floor of \
             {} (phase-392 W5.f).\n  \
             A service server IS a zenoh queryable. This image declares the \
             infrastructure services and application service servers that claim those \
             slots at boot, so a smaller table cannot start — it fails with \
             `ServiceServerCreationFailed` and no explanation (issue 0460).\n  \
             On Zephyr this knob is CONFIG_NROS_MAX_QUERYABLES, whose Kconfig default \
             is 8; raise it, or stop declaring the services the image does not have.",
            sizing.floor
        );
    }
    if requested < sizing.default {
        println!(
            "cargo:warning=ZPICO_MAX_QUERYABLES={requested} is under the budgeted {} \
             ({} provably claimed at boot, plus {} of headroom for application service \
             servers the model does not describe). It will boot; an application \
             declaring more than {} service servers will not (phase-392 W5.f).",
            sizing.default,
            sizing.floor,
            sizing.default - sizing.floor,
            requested.saturating_sub(sizing.floor),
        );
    }
}

/// What `NROS_DECLARED_INFRA_QUERYABLES` costs, in slots.
///
/// ONE parser, shared by the default and the floor: two readings of the same
/// spelling is how a sizing rule and its check come to disagree.
fn infra_queryables(infra: Option<&str>) -> usize {
    match infra {
        Some("none") => 0,
        Some("param") => PARAM_SERVICE_QUERYABLES,
        Some("lifecycle") => LIFECYCLE_SERVICE_QUERYABLES,
        Some("param+lifecycle") | Some("all") => {
            PARAM_SERVICE_QUERYABLES + LIFECYCLE_SERVICE_QUERYABLES
        }
        Some(other) => panic!(
            "NROS_DECLARED_INFRA_QUERYABLES={other:?} is not one of \
             none|param|lifecycle|param+lifecycle (phase-392 W5)."
        ),
        // Undeclared infrastructure is assumed PRESENT: over-reserving costs
        // RAM, under-reserving fails at boot with an exhausted table.
        None => PARAM_SERVICE_QUERYABLES + LIFECYCLE_SERVICE_QUERYABLES,
    }
}

/// The rule, with the environment lifted out so it can be tested.
///
/// A build script reading env directly is untestable in-process (env is
/// global), which is how a sizing rule ends up verified by reading.
fn queryable_default_from(declared: Option<&str>, infra: Option<&str>, hosted: bool) -> usize {
    let app = match declared {
        Some(v) => match v.trim().parse::<usize>() {
            Ok(n) => n,
            // A malformed declaration must not silently become "undeclared":
            // that is the `.max(1)` shape issue 0827 measured, where a value
            // reads as applied and is not.
            Err(_) => panic!(
                "NROS_DECLARED_SERVICE_SERVERS={v:?} is not a count. It is the \
                 number of service servers the entry declares, resolved from the \
                 SystemModel (phase-392 W5)."
            ),
        },
        // phase-392 W5.b1 — the model answered the INFRASTRUCTURE half and not
        // the application half, which is the normal case and not a broken
        // channel: `ros-launch-manifest` models service wiring, but the
        // resolver only emits it when the launch inputs describe endpoints,
        // and a plain `<node>` element does not — wiring comes from an AUTHORED
        // `<stem>.contract.yaml` beside the launch file (issue 0973). Measured
        // 2026-09-06: 109 of the tree's 114 resolvable models are that shape,
        // the other 5 having a contract. `nros ws entity-facts` therefore ABSTAINS
        // rather than reporting a zero it cannot support — reporting 0 for a
        // node called `add_server` would size the table to the infrastructure
        // alone and exhaust it at registration.
        //
        // So: keep the app headroom, but stop paying for infrastructure the
        // image demonstrably does not carry. On a native talker that is 32 -> 8
        // slots, which is most of the pool.
        None if infra.is_some() => UNDECLARED_HEADROOM,
        // Nothing was delivered at all — no model, no cmake seam, a bare
        // `cargo build`. The historical budgets, unchanged. W5.f retires both
        // once every image declares; that needs the hand-written-`main`
        // question settled first (phase-392 W5, Open).
        None => return if hosted { 32 } else { UNDECLARED_HEADROOM },
    };

    let infra = infra_queryables(infra);

    // A table of zero would make every service-server registration fail, so an
    // entry declaring none still gets one slot rather than a pool nothing can
    // index.
    core::cmp::max(app + infra, 1)
}

#[cfg(test)]
mod queryable_default_tests {
    use super::*;

    /// phase-392 W5.f — the floor is DERIVED, and it is not the default.
    #[test]
    fn the_floor_counts_only_what_is_provably_claimed() {
        // Undeclared: nothing is known, so nothing is provable.
        assert_eq!(queryable_floor_from(None, None), 0);
        // Infrastructure declared absent: an image that declares nothing still
        // needs nothing, even though its BUDGET is 8.
        assert_eq!(queryable_floor_from(None, Some("none")), 0);
        assert_eq!(queryable_default_from(None, Some("none"), true), 8);
        // Both service families: eleven slots claimed at boot. This is the
        // number issue 0460 discovered at runtime against a table of 8.
        assert_eq!(
            queryable_floor_from(None, Some("param+lifecycle")),
            PARAM_SERVICE_QUERYABLES + LIFECYCLE_SERVICE_QUERYABLES
        );
        // A declared application count is provable too — it is what the model
        // says the image will create.
        assert_eq!(queryable_floor_from(Some("2"), Some("lifecycle")), 7);
    }

    fn sizing(declared: Option<&str>, infra: Option<&str>) -> QueryableSizing {
        QueryableSizing {
            default: queryable_default_from(declared, infra, true),
            floor: queryable_floor_from(declared, infra),
            declared: declared.is_some() || infra.is_some(),
        }
    }

    #[test]
    #[should_panic(expected = "below this image's DERIVED floor")]
    fn an_override_under_the_derived_floor_is_refused() {
        // zephyr/Kconfig's `default 8` for CONFIG_NROS_MAX_QUERYABLES against
        // an entry carrying both service families: issue 0460's build, caught
        // at build time instead of at boot.
        check_queryable_override(8, &sizing(None, Some("param+lifecycle")));
    }

    #[test]
    fn an_override_at_the_floor_is_accepted() {
        // Exactly enough to boot. It is tight, not wrong.
        check_queryable_override(11, &sizing(None, Some("param+lifecycle")));
    }

    #[test]
    fn an_override_under_the_budget_is_only_reported() {
        // The three `workspaces/features` zephyr entries: 16 against a budgeted
        // 19, of which 11 is provable and 8 is headroom for an application
        // count no model here supplies. Refusing this would be refusing a build
        // for being under a guess.
        check_queryable_override(16, &sizing(None, Some("param+lifecycle")));
    }

    #[test]
    fn an_undeclared_image_is_not_checked_at_all() {
        // Nothing was declared, so nothing is known — the historical budgets
        // apply and any override is the caller's business, exactly as before.
        check_queryable_override(1, &sizing(None, None));
    }

    /// phase-392 W5.b1 — the half the model CAN answer, on its own.
    ///
    /// This is the shape every resolved model in the tree produces: the
    /// infrastructure flags are known (they are `execution.features`), the
    /// application's own service-server count is not (no resolver here emits
    /// layer-1 wiring). The app term stays a labelled headroom constant; the
    /// infrastructure term becomes exact.
    #[test]
    fn infra_declared_without_an_app_count_still_drops_the_hosted_guess() {
        // A native talker: no parameter services, no lifecycle. 32 -> 8.
        assert_eq!(
            queryable_default_from(None, Some("none"), true),
            UNDECLARED_HEADROOM
        );
        // An image that carries both: the headroom PLUS what they cost.
        assert_eq!(
            queryable_default_from(None, Some("param+lifecycle"), true),
            UNDECLARED_HEADROOM + PARAM_SERVICE_QUERYABLES + LIFECYCLE_SERVICE_QUERYABLES
        );
        // And it is the same number on an embedded target: the hosted/embedded
        // sniff decides nothing once the declaration arrives.
        assert_eq!(
            queryable_default_from(None, Some("none"), false),
            queryable_default_from(None, Some("none"), true)
        );
    }

    /// An unknown infrastructure spelling is a BROKEN CHANNEL, not an unknown
    /// feature — `nros ws entity-facts` emits one of four strings and drops
    /// names it does not recognise (`safety`) before it gets here.
    #[test]
    #[should_panic(expected = "NROS_DECLARED_INFRA_QUERYABLES")]
    fn an_unknown_infra_spelling_panics_even_without_an_app_count() {
        queryable_default_from(None, Some("safety"), true);
    }

    #[test]
    fn undeclared_keeps_the_historical_budgets() {
        assert_eq!(queryable_default_from(None, None, true), 32);
        assert_eq!(
            queryable_default_from(None, None, false),
            UNDECLARED_HEADROOM
        );
    }

    #[test]
    fn a_declaration_beats_the_hosted_guess() {
        // The talker case: no services, no infrastructure. Measured at 4,504
        // bytes of SERVICE_BUFFERS against 144,128 for the guess.
        assert_eq!(queryable_default_from(Some("0"), Some("none"), true), 1);
    }

    #[test]
    fn infrastructure_is_added_here_not_by_the_declarer() {
        // Issue 0460: codegen sees the user's entities and never the runtime's.
        assert_eq!(
            queryable_default_from(Some("0"), Some("param+lifecycle"), true),
            11
        );
        assert_eq!(
            queryable_default_from(Some("2"), Some("param+lifecycle"), true),
            13
        );
        assert_eq!(queryable_default_from(Some("2"), Some("param"), true), 8);
        assert_eq!(
            queryable_default_from(Some("2"), Some("lifecycle"), true),
            7
        );
    }

    #[test]
    fn unknown_infrastructure_is_assumed_present() {
        // Over-reserving wastes RAM; under-reserving fails at boot.
        assert_eq!(queryable_default_from(Some("0"), None, true), 11);
    }

    #[test]
    fn the_hosted_split_stops_mattering_once_declared() {
        assert_eq!(
            queryable_default_from(Some("3"), Some("none"), true),
            queryable_default_from(Some("3"), Some("none"), false)
        );
    }

    #[test]
    #[should_panic(expected = "is not a count")]
    fn a_malformed_count_is_not_silently_undeclared() {
        queryable_default_from(Some("lots"), Some("none"), true);
    }

    #[test]
    #[should_panic(expected = "is not one of")]
    fn an_unknown_infrastructure_spelling_is_refused() {
        queryable_default_from(Some("0"), Some("params"), true);
    }
}

fn shim_config_from_env() -> ShimConfig {
    // issue 0406 — these tables are STATIC arrays in the C shim, so every slot
    // costs RAM whether or not it is used. 8 is an embedded budget, and it was
    // applied to hosted targets too, where the same RAM argument does not hold.
    //
    // A service server IS a queryable, and the runtime registers its own
    // before the application declares anything — so on a hosted target the
    // 8-slot table overflowed at boot and every entry in
    // `examples/workspaces/features` died with a bare
    // `Transport(ServiceServerCreationFailed)`.
    //
    // Hosted targets get headroom; everything [`target_os_is_hosted`] calls
    // embedded keeps the embedded budget exactly as before. That predicate is
    // NOT `target_os != "none"` — issue 1028 measured that spelling handing
    // NuttX the 32-slot Linux budget. Override with the env var on either side.
    //
    // issue 0827 — this comment used to say "(6)" and "(6)" and "needs
    // twelve". Lifecycle is FIVE, so it is eleven, and "twelve" propagated
    // from here into other prose. The counts are now
    // `nros_node::parameter_services::PARAM_SERVICE_QUERYABLES` and
    // `nros_node::lifecycle_services::LIFECYCLE_SERVICE_QUERYABLES`, beside
    // the code that creates them. This crate cannot READ them — a build script
    // sees neither another crate's constants nor its features — which is
    // exactly why `32` below is a guess rather than a derivation, and why it
    // costs a hosted image 144,128 bytes of service buffers whether or not it
    // has a single service. Replacing the guess needs the declaration to reach
    // here from the resolved model; see issue 0827.
    let sizing = resolve_queryable_default();
    let max_queryables = env_usize("ZPICO_MAX_QUERYABLES", sizing.default);
    check_queryable_override(max_queryables, &sizing);
    ShimConfig {
        max_publishers: env_usize("ZPICO_MAX_PUBLISHERS", 8),
        max_subscribers: env_usize("ZPICO_MAX_SUBSCRIBERS", 8),
        max_queryables,
        queryable_table_declared: sizing.declared,
        max_liveliness: env_usize("ZPICO_MAX_LIVELINESS", 16),
        max_pending_gets: env_usize("ZPICO_MAX_PENDING_GETS", 4),
        max_sessions: env_usize("ZPICO_MAX_SESSIONS", 1),
        // phase-400 W6 — BUILTINS, like the tx pair below: these five are
        // ladder knobs now, and `resolve_wire` overwrites them where the rungs
        // are known (search `shim_config.get_reply_buf_size =`). Reading the
        // environment twice is what `check-knob-single-reader` forbids.
        get_reply_buf_size: 4096,
        get_poll_interval_ms: 10,
        // phase-400 W8 — the BUILTINS, not a second env read. These two are
        // ladder knobs, and the resolved values overwrite them below (search
        // `shim_config.tx_batch =`), so reading the environment here was dead
        // work that only looked authoritative. It is exactly the second reader
        // `check-knob-single-reader` forbids: harmless while the overwrite
        // stays put, a silent disagreement the moment it moves.
        tx_batch: platform_config::BUILTIN_TX_BATCH,
        tx_batch_flush_ms: platform_config::BUILTIN_TX_FLUSH_MS as usize,
        // Defaults MIRROR the `#define` fallbacks in
        // `zpico-sys/c/zpico/zpico.c` (16 / 16). A different number here would
        // not be a tuning choice, it would be the two lanes disagreeing.
        read_task_priority: env_usize("ZPICO_READ_TASK_PRIORITY", 16),
        lease_task_priority: env_usize("ZPICO_LEASE_TASK_PRIORITY", 16),
    }
}

/// Read buffer config from environment variables with platform-appropriate defaults.
fn zenoh_buffer_config_from_env(posix: bool) -> ZenohBufferConfig {
    // Phase 204.7 — `NROS_LINK_IP=0` (a serial-only node) gates the IP link
    // C off; rerun the build script when it changes.
    println!("cargo:rerun-if-env-changed=NROS_LINK_IP");
    let link = LinkFeatures::from_env();
    let (default_frag, default_batch_uni, default_batch_multi) = if posix {
        (65535, 65535, 8192)
    } else if link.serial {
        (2048, 1500, 1024)
    } else {
        (2048, 1024, 1024)
    };

    ZenohBufferConfig {
        // phase-400 W6 — the computed per-platform BUILTINS. `resolve_wire`
        // takes them as its `defaults` and applies the rungs above them, so the
        // transport still picks the starting number and a descriptor can still
        // override it.
        frag_max_size: default_frag,
        batch_unicast_size: default_batch_uni,
        batch_multicast_size: default_batch_multi,
    }
}

/// The Kconfig option each shim knob is resolved from on Zephyr.
///
/// issue 0460 — a Zephyr RUST image never sees the `set(ENV{...})` exports that
/// `nros_cargo_build.cmake` writes (zephyr-lang-rust's `rust_cargo_application`
/// builds its own cargo command and inherits nothing), so a knob set in
/// Kconfig reached the C lane and not this one. `ZPICO_MAX_QUERYABLES` is where
/// that stopped being invisible: Kconfig said 16, the cmake-compiled shim TU
/// got 16, this build script compiled the crate default of 8, and the Rust-side
/// slot guard — whose explanatory log is `cfg(feature = "std")`, i.e. silent on
/// every embedded image — refused the ninth queryable. The three
/// `workspaces/features` zephyr entries register eleven capability services
/// (six param + five lifecycle) and died there with a bare
/// `Transport(ServiceServerCreationFailed)` and no other output.
///
/// The pairs are NOT derivable (`ZPICO_TX_BATCH_FLUSH_MS` comes from
/// `CONFIG_NROS_ZENOH_TX_BATCH_FLUSH_MS`, not `CONFIG_NROS_TX_BATCH_FLUSH_MS`),
/// so they are declared. `check-kconfig-knob-forwarding` holds this table and
/// `_nros_resolve_knob()` in `nros_cargo_build.cmake` to the same set: a knob
/// in the cmake list and not here is one more silently-defaulted image.
const KCONFIG_KNOBS: &[(&str, &str)] = &[
    ("ZPICO_MAX_PUBLISHERS", "CONFIG_NROS_MAX_PUBLISHERS"),
    ("ZPICO_MAX_SUBSCRIBERS", "CONFIG_NROS_MAX_SUBSCRIBERS"),
    ("ZPICO_MAX_QUERYABLES", "CONFIG_NROS_MAX_QUERYABLES"),
    ("ZPICO_MAX_LIVELINESS", "CONFIG_NROS_MAX_LIVELINESS"),
    ("ZPICO_MAX_PENDING_GETS", "CONFIG_NROS_MAX_PENDING_GETS"),
    ("ZPICO_GET_REPLY_BUF_SIZE", "CONFIG_NROS_GET_REPLY_BUF_SIZE"),
    (
        "ZPICO_GET_POLL_INTERVAL_MS",
        "CONFIG_NROS_GET_POLL_INTERVAL_MS",
    ),
    ("ZPICO_FRAG_MAX_SIZE", "CONFIG_NROS_FRAG_MAX_SIZE"),
    ("ZPICO_BATCH_UNICAST_SIZE", "CONFIG_NROS_BATCH_UNICAST_SIZE"),
    ("ZPICO_TX_BATCH", "CONFIG_NROS_ZENOH_TX_BATCH"),
    ("ZPICO_TX_SPLIT_LOCK", "CONFIG_NROS_ZENOH_TX_SPLIT_LOCK"),
    (
        "ZPICO_TX_BATCH_FLUSH_MS",
        "CONFIG_NROS_ZENOH_TX_BATCH_FLUSH_MS",
    ),
    // issue 0626's knobs. The cmake side forwarded them from the day the
    // feature landed; without these rows only the C lane saw them, which is
    // issue 0460 exactly.
    (
        "ZPICO_READ_TASK_PRIORITY",
        "CONFIG_NROS_ZENOH_READ_PRIORITY",
    ),
    (
        "ZPICO_LEASE_TASK_PRIORITY",
        "CONFIG_NROS_ZENOH_LEASE_PRIORITY",
    ),
];

/// A knob's value as a STRING: explicit env, else Kconfig, else `None`.
///
/// The tx trio is resolved through a board-toml ladder that takes an
/// `Option<String>` per knob rather than a default, so it needs this shape
/// instead of [`env_usize`]. Same table, same precedence.
fn kconfig_fallback_str(name: &str) -> Option<String> {
    if let Some(v) = env::var(name).ok().filter(|v| !v.is_empty()) {
        return Some(v);
    }
    let (_, kconfig) = KCONFIG_KNOBS.iter().find(|(env, _)| *env == name)?;
    nros_zephyr_build::dotconfig_usize(kconfig).map(|v| v.to_string())
}

/// Read a usize knob: explicit env var, else Zephyr Kconfig, else `default`.
///
/// A knob with no [`KCONFIG_KNOBS`] row (`ZPICO_MAX_SESSIONS`,
/// `ZPICO_BATCH_MULTICAST_SIZE` — neither is exposed as Kconfig) keeps the
/// plain env-or-default behaviour.
fn env_usize(name: &str, default: usize) -> usize {
    match KCONFIG_KNOBS.iter().find(|(env, _)| *env == name) {
        Some((_, kconfig)) => nros_zephyr_build::knob_usize(name, kconfig, default),
        None => {
            println!("cargo:rerun-if-env-changed={name}");
            env::var(name)
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(default)
        }
    }
}

/// Generate a zenoh-pico config header in OUT_DIR based on Cargo link-* features.
///
/// This replaces the hardcoded `zenoh_generic_config.h` with a generated version
/// where `Z_FEATURE_LINK_*` values are derived from Cargo features and buffer
/// sizes are read from environment variables with platform-appropriate defaults.
/// phase-290 — tx flags arrive RESOLVED through the RFC-0049 ladder
/// (builtin < platform toml < board toml < env) instead of raw env reads,
/// so the platform default (e.g. zephyr batch-on) reaches the ABI-gating
/// `Z_FEATURE_*` header while an explicit env `0` still wins.
/// Every `nros-platform.toml` that EXISTS under the platform search path.
///
/// Issue 1039 follow-up. This used to reconstruct the path as
/// `root/<platform-name>/nros-platform.toml`, which is not the layout: a
/// platform package directory is `nros-platform-<name>`, so
/// `packages/platform/nuttx/nros-platform.toml` never existed and the
/// `is_file()` filter dropped it. The manifests under `config/<name>/` matched
/// and were watched; every manifest under `packages/platform/` was NOT.
///
/// The consequence is worse than a missed rebuild, because it is silent: an
/// edit to `packages/platform/nros-platform-nuttx/nros-platform.toml` changed
/// nothing until something unrelated forced the build script to re-run. An
/// experiment on platform defines then measures the OLD defines and reads as a
/// refuted hypothesis. That happened while diagnosing issue 1039 and cost a
/// wrong conclusion, which is the same shape as every other staleness defect
/// here: a stale input that reports as a result.
///
/// Enumerating directories rather than reconstructing names also removes the
/// dependence on a name matching its directory at all — the thing that broke.
/// Watching a manifest that exists but is SHADOWED by a higher-priority root is
/// deliberate and harmless: it costs an occasional rebuild and cannot miss one.
///
/// Still only paths that EXIST (issue 0966): cargo treats a missing
/// `rerun-if-changed` input as permanently dirty, which silently rebuilds
/// `zpico-sys` and everything above it on every invocation.
fn platform_manifests_to_watch(roots: &[PathBuf]) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for root in roots {
        let Ok(entries) = std::fs::read_dir(root) else {
            continue; // a missing root is normal for a search PATH
        };
        let mut found: Vec<PathBuf> = entries
            .filter_map(|e| e.ok())
            .map(|e| e.path())
            .filter(|p| p.is_dir())
            .map(|d| d.join(platform_config::PLATFORM_CONFIG_FILENAME))
            .filter(|m| m.is_file())
            .collect();
        found.sort();
        out.append(&mut found);
    }
    out
}

fn generate_config_header(
    out_dir: &Path,
    link: &LinkFeatures,
    buf: &ZenohBufferConfig,
    tx_batch: bool,
    tx_split_lock: bool,
) {
    let config_dir = out_dir.join("zenoh-config");
    std::fs::create_dir_all(&config_dir).unwrap();
    let target = std::env::var("TARGET").unwrap_or_default();
    let header = crate::config_header(
        link,
        buf,
        &target,
        env::var("CARGO_FEATURE_UNSTABLE_ZENOH_API").is_ok(),
        tx_batch,
        tx_split_lock,
    );
    std::fs::write(config_dir.join("zenoh_generic_config.h"), header).unwrap();
}

pub fn run() {
    // Issue 1005 — THIS crate's sources are an input to every build that calls
    // `run()`, and until now nothing said so. Cargo tracks the dependency
    // correctly through its own unit graph, so an edit here does rebuild the
    // consumer; what was missing is a `rerun-if-changed` PATH, and the fixture
    // staleness probe reads exactly that recorded path list
    // (`zpico_recorded_inputs`). The consequence was measured: `Z_TRANSPORT_LEASE_MS`
    // moved 10_000 -> 60_000 on 2026-08-30 (issue 0906) and every FreeRTOS
    // fixture in the tree still baked 10000 ten days later while the probe
    // reported FRESH -- a known, measured, delivery-breaking value behind a
    // green verdict.
    //
    // Emitted HERE and not from each consuming `build.rs` on purpose. The whole
    // build-script implementation lives in this crate (`zpico-sys/build.rs` is
    // a three-line shim), so a future consumer inherits the input declaration
    // instead of having to remember it -- and "every consumer remembers" is the
    // property that already failed.
    //
    // `env!` binds THIS crate's manifest dir at ITS compile time, which is what
    // makes the path right no matter who calls `run()`. Watching a PATH is the
    // safe direction: cargo reads a `rerun-if-changed` back from the stored
    // output and never re-resolves it, unlike `rerun-if-env-changed` on a path
    // VALUE (issue 0491).
    println!("cargo:rerun-if-changed={}/src", env!("CARGO_MANIFEST_DIR"));

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let target = env::var("TARGET").unwrap_or_default();
    // 192.3: first-party include located via env (default in sdk-env.just),
    // not a build.rs walk-up — used in the platform-gated C-build paths below.
    //
    // issue 0491 — watched by CONTENT, never fingerprinted as an env STRING:
    // cargo compares that value textually, and an example leaf spells it
    // `relative = true` (one spelling per leaf) while `just` exports it
    // absolute, so rows sharing one `--target-dir` re-ran this script forever.
    nros_build_paths::watch_path(&nros_build_paths::nros_platform_cffi_include());

    // Phase 136.1 — parse the canonical platform manifest. Resolve
    // every declared platform so a typo or broken `inherits` chain
    // surfaces as a hard build error, not a runtime surprise after
    // 136.3 plugs the data into cc-rs.
    //
    // Phase 136.7-E2E.3 — `ZPICO_PLATFORMS_TOML` env var redirects
    // the manifest to a caller-supplied path. Used by the drift-gate
    // test (`tests/zpico_drift_gate.rs`) to point at sandboxed
    // manifests; also a documented out-of-tree override hook for
    // downstream boards. Empty value falls through to the canonical
    // in-tree manifest.
    // phase-290 (RFC-0049) — the central `zenoh_platforms.toml` is retired.
    // Each platform package directory carries `nros-platform.toml` with the
    // `[build.zenoh]` block (keys verbatim) + `[capabilities]` + `[knobs]`.
    // Precedence: `ZPICO_PLATFORMS_TOML` (legacy single-file override — the
    // drift-gate test + out-of-tree hook) > `NROS_PLATFORMS_DIR` (tree
    // override) > the in-tree `config/` default.
    // Both selectors are PATHS, so neither is fingerprinted as a string
    // (issue 0491) — the manifest FILES they select are watched by
    // `rerun-if-changed` in both arms of the match below.
    let legacy_file = env::var_os("ZPICO_PLATFORMS_TOML")
        .filter(|v| !v.is_empty())
        .map(PathBuf::from);
    // phase-400 W1 — a SEARCH PATH, not one directory.
    //
    // This used to be `manifest_dir.join("../../../../config")`, a single
    // hardcoded root. When the descriptors moved beside their crates
    // (`packages/platform/nros-platform-<x>/`) that path still resolved — to a
    // directory that no longer held them — so every platform silently fell back
    // to builtins. A wrong image, no diagnostic: exactly the silent-fallback
    // failure phase-400 W8 exists to prevent, and it would have shipped.
    //
    // `NROS_PLATFORMS_DIR` keeps working as an explicit single-root override by
    // being placed FIRST in the path; it is no longer the only way in.
    let repo_root = manifest_dir.join("../../../..");
    let platform_search_path = platform_config::PlatformsTree::default_search_path(
        &repo_root,
        env::var("NROS_PLATFORMS_DIR").ok().as_deref(),
    );

    let (platform_manifest, platforms_tree) = match legacy_file {
        Some(path) => {
            println!("cargo:rerun-if-changed={}", path.display());
            let m = manifest::PlatformManifest::load(&path)
                .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
            (m, None)
        }
        None => {
            let tree = platform_config::PlatformsTree::load_search_path(&platform_search_path)
                .unwrap_or_else(|e| panic!("platform search path: {e}"));
            if tree.names().next().is_none() {
                panic!(
                    "no nros-platform.toml found on the platform search path: {}. \
                     A silently empty tree resolves every knob to a builtin and produces a \
                     wrong image with no diagnostic, so this is fatal rather than a warning.",
                    platform_search_path
                        .iter()
                        .map(|p| p.display().to_string())
                        .collect::<Vec<_>>()
                        .join(":")
                );
            }
            // Watch every root, not just the one a descriptor happens to live
            // in today: moving a file between roots must invalidate the build.
            // issue 0966 — emit only the manifests that EXIST.
            //
            // This is a cross product of (search roots) x (platform names), and
            // a given platform's manifest lives under exactly ONE root: with
            // `packages/platform` and `config` both on the path, `bare-metal`
            // (which lives in `config/`) also produced
            // `packages/platform/bare-metal/nros-platform.toml`, which does not
            // exist. Cargo treats a MISSING `rerun-if-changed` input as
            // permanently dirty, so that one path recompiled `zpico-sys` and
            // everything above it on EVERY invocation — silently, because the
            // build always succeeded, it was just never fresh
            // (`stale: missing .../packages/platform/bare-metal/nros-platform.toml`
            // under `CARGO_LOG=cargo::core::compiler::fingerprint=info`).
            // Same class as issue 0490.
            //
            // The trade in filtering: a manifest CREATED later under a
            // higher-priority root does not by itself trigger a rebuild. Cargo
            // cannot watch a nonexistent path without being permanently dirty,
            // so watching what exists is the only stable option, and authoring a
            // new platform descriptor is a deliberate act that comes with other
            // edits. `PlatformsTree` does not record which root each name was
            // loaded from; if it ever does, watch exactly those instead.
            for manifest in platform_manifests_to_watch(&platform_search_path) {
                println!("cargo:rerun-if-changed={}", manifest.display());
            }
            let m = tree.as_platform_manifest();
            (m, Some(tree))
        }
    };
    for name in platform_manifest.platform.keys() {
        platform_manifest
            .for_platform(name)
            .unwrap_or_else(|e| panic!("nros-platform.toml: resolve {name}: {e}"));
    }

    // Phase 136.6 (partial) — source-list drift gate. For every
    // platform, verify each `include` root names a real directory
    // under `zenoh-pico/src/` and contains at least one `.c` file.
    // Catches submodule bumps that rename / delete dirs and typos
    // in the manifest. Full set-equality vs. the cc-rs source list
    // lands with 136.4 once the per-RTOS functions collapse into a
    // single manifest-driven path.
    let zenoh_pico_src = manifest_dir.join("zenoh-pico").join("src");
    if zenoh_pico_src.exists() {
        for name in platform_manifest.platform.keys() {
            let resolved = platform_manifest.for_platform(name).unwrap();
            for include in &resolved.include {
                let dir = zenoh_pico_src.join(include);
                if !dir.is_dir() {
                    panic!(
                        "nros-platform.toml: platform `{name}` `include = \"{include}\"` \
                         does not resolve to a directory under zenoh-pico/src/ \
                         (expected: {})",
                        dir.display()
                    );
                }
                let has_c_file = std::fs::read_dir(&dir)
                    .map(|entries| {
                        entries.flatten().any(|e| {
                            e.path().extension().is_some_and(|x| x == "c")
                                || e.file_type().map(|t| t.is_dir()).unwrap_or(false)
                        })
                    })
                    .unwrap_or(false);
                if !has_c_file {
                    panic!(
                        "nros-platform.toml: platform `{name}` `include = \"{include}\"` \
                         resolves to {} but contains no .c files or subdirs",
                        dir.display()
                    );
                }
                println!("cargo:rerun-if-changed={}", dir.display());
            }
        }
    }

    // Check which platform backend to use
    let mut use_posix = env::var("CARGO_FEATURE_POSIX").is_ok();
    let use_zephyr = env::var("CARGO_FEATURE_ZEPHYR").is_ok();
    let use_bare_metal = env::var("CARGO_FEATURE_BARE_METAL").is_ok();
    let use_freertos = env::var("CARGO_FEATURE_FREERTOS").is_ok();
    let use_nuttx = env::var("CARGO_FEATURE_NUTTX").is_ok();
    let use_threadx = env::var("CARGO_FEATURE_THREADX").is_ok();

    // Phase 128.D — auto-derive `platform-posix` from `target_os` when
    // no explicit platform feature was selected. The POSIX path is
    // the only one a `cargo build` on a hosted target can infer
    // unambiguously; the embedded RTOSes still disambiguate by enabling
    // the matching feature. NOTE this used to say they "all share
    // `target_os = "none"`" — issue 1028: NuttX does not
    // (`armv7a-nuttx-eabihf` is `target_os = "nuttx"`,
    // `target_family = "unix"`). The inference below is safe anyway
    // because it is an ALLOWLIST of hosts rather than a `!= "none"`
    // test, which is exactly the shape 1028 wanted everywhere. The hosts on the
    // list below no longer need an explicit `platform-posix` feature in
    // their `Cargo.toml`. NOTE the list is Linux / *BSD / Android and does
    // NOT include macOS — this comment used to claim macOS as an
    // auto-detected POSIX host while the `matches!` never named it, so a
    // macOS build still has to pass the feature explicitly.
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let auto_posix = matches!(
        target_os.as_str(),
        "linux" | "freebsd" | "netbsd" | "openbsd" | "android"
    );
    let any_explicit =
        use_posix || use_zephyr || use_bare_metal || use_freertos || use_nuttx || use_threadx;
    if !any_explicit && auto_posix {
        use_posix = true;
    }

    // Issue 0919 — the question the alias TU needs answered is "is a platform
    // RESOLVED?", not "did the consumer name one?". `any_explicit` answers the
    // second, and is computed BEFORE the inference above, so it stays false on
    // a hosted target that resolved POSIX by inference. Gating code generation
    // on it therefore configured zenoh-pico FOR POSIX and then declined to emit
    // the platform forwarders that configuration requires.
    //
    // The result is 16 undefined symbols at LINK time in a consumer —
    // `z_sleep_ms`, `z_malloc`, `_z_mutex_*` — reported against whatever binary
    // happened to link zpico, with nothing pointing back at the crate that made
    // the decision. It took `host-tests` red on main and eight local routes that
    // did not reproduce, because reproducing it needs a graph where zpico is
    // present WITHOUT `platform-posix`: the entry depends on `nros-rmw-zenoh`
    // directly (so `platform-aliases` is on by default) while the board it goes
    // through selects only another RMW, so nothing enables the platform feature.
    //
    // A build that reaches here with no platform at all is a different case and
    // still not the alias TU's business: `backend_count` below rejects it.
    let platform_resolved =
        use_posix || use_zephyr || use_bare_metal || use_freertos || use_nuttx || use_threadx;

    // Count enabled backends
    let backend_count = [
        use_posix,
        use_zephyr,
        use_bare_metal,
        use_freertos,
        use_nuttx,
        use_threadx,
    ]
    .iter()
    .filter(|&&b| b)
    .count();

    if backend_count == 0 {
        // No backend selected — build only enough to regenerate `zpico.h`
        // (cbindgen) and the size probe. Reached on plain `cargo doc`,
        // `cargo check --workspace`, etc. — perfectly normal, so emit
        // an `eprintln!` instead of `cargo:warning` (the latter surfaces
        // as a yellow `warning: …` line on every workspace build).
        eprintln!("zpico-sys: no platform backend selected; minimal build (header-only).");
    }

    if backend_count > 1 {
        panic!(
            "Only one platform backend can be selected at a time \
             (posix, zephyr, bare-metal, freertos, nuttx, or threadx)"
        );
    }

    // Read link-* features for bare-metal protocol selection
    let link_features = LinkFeatures::from_env();

    // Phase 100.4 — surface the IVC link feature as a cfg so
    // `zpico-platform-shim` can gate its `mod ivc_helpers` block on
    // `#[cfg(feature = "link_ivc")]` without depending on the cargo
    // feature flag transitively. Mirrors the `zpico_backend` pattern
    // used a few hundred lines below.
    if link_features.ivc {
        println!("cargo:rustc-cfg=feature=\"link-ivc\"");
    }
    println!("cargo:rustc-check-cfg=cfg(feature, values(\"link-ivc\"))");

    // Paths
    let zenoh_pico_src = manifest_dir.join("zenoh-pico");
    let c_dir = manifest_dir.join("c");
    let include_dir = c_dir.join("include");
    let use_system = env::var("CARGO_FEATURE_SYSTEM_ZENOHPICO").is_ok();

    // system-zenohpico is not supported for embedded targets — embedded builds need
    // the smoltcp platform layer compiled into the same archive as zenoh-pico.
    if use_system && is_embedded_target(&target) {
        panic!(
            "system-zenohpico is not supported for embedded targets ({}). \
             Embedded builds require the smoltcp platform layer to be compiled \
             together with zenoh-pico from the submodule.",
            target
        );
    }

    // Check if zenoh-pico submodule is present (not needed with system-zenohpico)
    if !use_system && !zenoh_pico_src.join("include").exists() {
        panic!(
            "zenoh-pico source not provisioned at {:?}. Run: nros setup --source \
             zenoh-pico (or git submodule update --init \
             packages/rmw/zenoh/zpico-sys/zenoh-pico). — #0390",
            zenoh_pico_src
        );
    }

    // phase-400 W2a — the committed `zpico.h` is an INPUT to the C shim
    // compiled below, so its rebuild edge is unconditional; only the drift
    // COMPARISON is behind the feature.
    println!(
        "cargo:rerun-if-changed={}",
        include_dir.join("zpico.h").display()
    );
    #[cfg(feature = "cbindgen-drift-check")]
    generate_header(&manifest_dir, &include_dir);

    // phase-290 — pick the manifest platform up front so the RFC-0049 knob
    // ladder (builtin < platform toml < board toml < env) can resolve before
    // the ABI-gating config header is generated.
    // issue 0529 — `zephyr` was missing here, and every Zephyr target matches
    // `is_embedded_target()`, so the resolver returned None and the ladder fell
    // to builtins for the ONE platform file that carries `[knobs.zenoh.tx]`.
    //
    // No behaviour change today: `build_c_shim` is skipped on Zephyr (below), so
    // the config header these knobs feed has no consumer, and the C lane gets
    // the same values from Kconfig via `nros_rmw_zenoh.cmake`. This makes the
    // resolver TOTAL over the platforms that have a config file, so the next
    // knob added to that table is not silently ignored.
    let platform_name = if use_threadx {
        Some("threadx")
    } else if use_nuttx {
        Some("nuttx")
    } else if use_freertos {
        Some("freertos")
    } else if use_zephyr {
        Some("zephyr")
    } else if use_bare_metal {
        Some("bare-metal")
    } else if !is_embedded_target(&target) && !use_system {
        Some("posix")
    } else {
        None
    };

    // phase-290 — resolve the zenoh.tx knob set through the ladder. Board
    // deltas ride `NROS_BOARD_TOML` (a board package's nros-board.toml path,
    // exported by the build orchestration when a board is in play). In
    // legacy single-file mode (`ZPICO_PLATFORMS_TOML`) there is no knob
    // layer — env-only, byte-identical to pre-290.
    // `NROS_BOARD_TOML` is a PATH (issue 0491): the file it names is watched
    // with `rerun-if-changed` where it is loaded, a few lines below.
    // issue 0682 — the peer-mode build input. `multicast_transport_enabled()`
    // reads the env DIRECTLY rather than through `env_usize`, so it does not
    // emit its own rebuild edge; without this line the three C `#define`s and
    // `ZPICO_PEER_MODE_SUPPORTED` would keep whatever the last build baked, and
    // `just test-zpico-peer` would silently test the client-mode shim.
    println!(
        "cargo:rerun-if-env-changed={}",
        crate::MULTICAST_TRANSPORT_ENV
    );
    println!("cargo:rerun-if-env-changed=ZPICO_TX_BATCH");
    println!("cargo:rerun-if-env-changed=ZPICO_TX_SPLIT_LOCK");
    println!("cargo:rerun-if-env-changed=ZPICO_TX_BATCH_FLUSH_MS");
    // issue 0460 — the same env-or-Kconfig ladder the sized knobs use, so the
    // tx trio is not one more thing a Zephyr RUST image reads as "unset".
    let env_get = |name: &str| kconfig_fallback_str(name);
    // phase-400 W6 — loaded ONCE and shared by both zenoh tenants (`tx` and
    // `wire`). It used to live inside the tx arm; a second `load` for the wire
    // knobs would parse the same file twice and could disagree with itself if
    // one call site ever grew an option the other did not.
    let board_knobs = env_get("NROS_BOARD_TOML").map(|path| {
        let p = PathBuf::from(&path);
        println!("cargo:rerun-if-changed={}", p.display());
        platform_config::BoardKnobsFile::load(&p).unwrap_or_else(|e| panic!("{path}: {e}"))
    });
    let tx_knobs = match (&platforms_tree, platform_name) {
        (Some(tree), Some(name)) => {
            let mut tx = tree
                .resolve_tx(
                    name,
                    board_knobs.as_ref().map(|b| &b.knobs.zenoh.tx),
                    &env_get,
                )
                .unwrap_or_else(|e| panic!("nros-platform.toml: {e}"));
            for w in tree
                .capability_check(name, &mut tx)
                .unwrap_or_else(|e| panic!("nros-platform.toml: {e}"))
            {
                println!("cargo:warning={w}");
            }
            tx
        }
        _ => {
            // Legacy / unknown-platform fallback: the env front-end over the
            // builtins, resolved by the ONE implementation of that rule.
            //
            // phase-400 W8 — this used to re-read the three knobs here, which
            // made each of them have two readers that could disagree, and
            // reported `KnobSource::Env` even for a value that came from a
            // builtin. `check-knob-single-reader` now derives its list from the
            // census, which is how the pair was found.
            platform_config::tx_env_only(&env_get)
        }
    };

    // Read buffer config with platform-appropriate defaults
    // NuttX is POSIX-compatible, use same defaults as posix.
    // ThreadX uses NetX Duo BSD sockets, treat as posix-like for buffer defaults.
    let mut buf_config = zenoh_buffer_config_from_env(use_posix || use_nuttx || use_threadx);

    // Read shim slot counts from ZPICO_MAX_* env vars and generate Rust consts
    let mut shim_config = shim_config_from_env();
    // phase-290 — the env reads inside shim_config_from_env are the raw
    // top-rung values; overwrite the tx pair with the ladder-resolved ones.
    shim_config.tx_batch = tx_knobs.batch.value;
    shim_config.tx_batch_flush_ms = tx_knobs.flush_ms.value as usize;
    // phase-400 W6 — the zenoh WIRE sizes, same ladder. The builtins are the
    // values `shim_config_from_env` just computed from the platform's
    // transport, so a board that says nothing keeps the transport-appropriate
    // number and a board that speaks outranks it.
    {
        let wire_defaults: [(&'static str, usize); 5] = [
            ("batch_unicast_size", buf_config.batch_unicast_size),
            ("batch_multicast_size", buf_config.batch_multicast_size),
            ("frag_max_size", buf_config.frag_max_size),
            ("get_reply_buf_size", shim_config.get_reply_buf_size),
            ("get_poll_interval_ms", shim_config.get_poll_interval_ms),
        ];
        for key in platform_config::WIRE_KNOBS {
            println!(
                "cargo:rerun-if-env-changed={}",
                platform_config::wire_env_key(key)
            );
        }
        let resolved = match (&platforms_tree, platform_name) {
            (Some(tree), Some(name)) => tree
                .resolve_wire(
                    name,
                    board_knobs.as_ref().map(|b| &b.knobs.zenoh.wire),
                    &env_get,
                    &wire_defaults,
                )
                .unwrap_or_else(|e| panic!("nros-platform.toml: {e}")),
            // No platform named: the env front-end over the computed builtins,
            // which is what an out-of-tree consumer and a plain `cargo build`
            // get.
            _ => wire_defaults
                .iter()
                .map(|(name, builtin)| {
                    let key = platform_config::wire_env_key(name);
                    let value = env_get(key)
                        .and_then(|v| v.trim().parse::<usize>().ok())
                        .unwrap_or(*builtin);
                    (*name, value)
                })
                .map(|(name, value)| {
                    (
                        name,
                        platform_config::ResolvedUsize {
                            value,
                            source: platform_config::KnobSource::Builtin,
                            env_key: platform_config::wire_env_key(name),
                        },
                    )
                })
                .collect(),
        };
        for (name, r) in resolved {
            match name {
                "batch_unicast_size" => buf_config.batch_unicast_size = r.value,
                "batch_multicast_size" => buf_config.batch_multicast_size = r.value,
                "frag_max_size" => buf_config.frag_max_size = r.value,
                "get_reply_buf_size" => shim_config.get_reply_buf_size = r.value,
                "get_poll_interval_ms" => shim_config.get_poll_interval_ms = r.value,
                other => unreachable!("unknown wire knob `{other}`"),
            }
        }
    }
    std::fs::write(out_dir.join("shim_constants.rs"), shim_config.rust_consts()).unwrap();

    // Phase 134.3 — `zenoh_generic_config.h` is the single source of
    // truth for every `Z_FEATURE_LINK_*` flag. Apply the per-platform
    // `LinkPolicy` once here, generate the canonical header, and every
    // build path below reads it (cc-rs via `ZENOH_GENERIC` + include
    // path; CMake via the same — see 134.4 below).
    let link_policy = if use_posix {
        LinkPolicy::posix()
    } else if use_freertos {
        // Phase 146.2 — FreeRTOS has no serial / raweth / TLS
        // backend; force them off so the upstream "Serial not
        // supported" `#error` doesn't fire and the alias TU
        // doesn't have to stub `_z_*_serial_internal`.
        LinkPolicy::freertos_lwip()
    } else if use_nuttx {
        // Phase 146.2 — NuttX has no serial / raweth / TLS
        // backend either. Same shape as freertos_lwip().
        LinkPolicy::nuttx()
    } else if use_threadx {
        // Phase 146.2 — ThreadX uses platform_aliases.c for
        // network ops (no serial wrapper there); force serial
        // off to skip building zenoh-pico's serial.c against
        // undefined `_z_*_serial_internal` symbols.
        LinkPolicy::threadx()
    } else {
        LinkPolicy::passthrough()
    };
    // issue 1143 — the BOARD rung. The chain above answers "which network
    // backend does this PLATFORM's system TU set provide", which is the right
    // question for a family and the wrong one for a member of it: the FreeRTOS
    // family holds both an lwIP board and (phase-418) a mailbox-only one, and
    // before this the second could not be expressed at all —
    // `LinkPolicy::ivc_only()` had existed, correct and unreachable, since the
    // SPE board left.
    //
    // The fact is `capabilities.ip_stack` (RFC-0086), resolved through the
    // RFC-0049 ladder with the board's own `[capabilities]` over its
    // platform's. Absent leaves the platform policy exactly as it was, so a
    // build with no `NROS_BOARD_TOML` — a plain `cargo build -p zpico-sys`, an
    // out-of-tree consumer — is byte-identical to before. See
    // `LinkPolicy::for_board` for why `supported_netstacks = []` is NOT the
    // predicate: four in-tree boards declare it and all four have sockets.
    //
    // `NROS_BOARD` rides beside `NROS_BOARD_TOML` from the same seam
    // (`nros board-facts`) and names WHICH `[[board]]` when a file declares
    // several — `packages/boards/nros-board-nuttx` declares two that differ in
    // ISA, so a file-level answer would be issue 0606 one table over.
    println!("cargo:rerun-if-env-changed=NROS_BOARD");
    let board_name = env_get("NROS_BOARD");
    let board_capabilities = match (&platforms_tree, platform_name) {
        (Some(tree), Some(name)) => tree
            .capabilities_with_board(name, board_knobs.as_ref(), board_name.as_deref())
            .unwrap_or_else(|e| panic!("nros-board.toml capabilities: {e}")),
        _ => std::collections::BTreeMap::new(),
    };
    let link_policy = LinkPolicy::for_board(link_policy, &board_capabilities);
    let link_features = link_features.apply(&link_policy);
    generate_config_header(
        &out_dir,
        &link_features,
        &buf_config,
        tx_knobs.batch.value,
        tx_knobs.split_lock.value,
    );

    // Phase 136.4 — manifest-driven unified consumer. The TOML
    // declares every per-platform datum (defines, required env
    // vars, include paths, extra sources, arch profile, compile
    // settings, pic). Drop the five per-RTOS Rust functions in
    // favour of a single `build_zenoh_pico_unified` that consumes
    // `ResolvedPlatform` + `[arch.*]`.
    let interp_ctx = manifest::InterpContext {
        nros: &manifest_dir,
        out: &out_dir,
        src: &zenoh_pico_src.join("src"),
    };
    // phase-290 — platform_name computed above (knob resolution).
    //
    // issue 0541 — `!use_zephyr`, because `platform_name` gates TWO things and
    // #0529 only reasoned about one. That change made the name total over the
    // platforms with a config file (right, for the knob ladder) and argued "no
    // behaviour change today: `build_c_shim` is skipped on Zephyr" — true, and
    // beside the point: this call is not the shim, it is the whole vendored
    // zenoh-pico C build. Before #0529 Zephyr resolved to `None` and cargo
    // compiled NO zenoh-pico C; after it, cargo compiled the core with the
    // Zephyr platform header and died on
    //
    //     zenoh-pico/system/platform/zephyr.h:18:10:
    //       fatal error: version.h: No such file or directory
    //
    // `version.h` is GENERATED by the Zephyr build, and on Zephyr the C sources
    // belong to the cmake module (`zpico-zephyr`), which supplies the generated
    // include dirs via `zephyr_include_directories`. The cargo lane has no such
    // path and must not build them — it never did.
    if let Some(name) = platform_name.filter(|_| !use_zephyr) {
        let resolved = platform_manifest
            .for_platform(name)
            .unwrap_or_else(|e| panic!("nros-platform.toml: {e}"));
        // issue 0534 — a platform whose own build system compiles zenoh-pico
        // declares `compiled_by = "platform"`, and cargo must not compile it:
        // its vendored `system/<platform>/*.c` can need headers that only that
        // build system generates (Zephyr's `version.h`). The block is still
        // resolved above — the knobs, defines and includes it carries are read
        // by the drift gate and document the external build.
        if resolved.compiled_by == manifest::CompiledBy::Cargo {
            build_zenoh_pico_unified(
                &resolved,
                &platform_manifest.arch,
                &interp_ctx,
                &zenoh_pico_src,
                &out_dir,
                &target,
                &link_features,
                &shim_config,
                &board_capabilities,
            );
        }
    }

    // POSIX still needs the separate C shim build below (shim is
    // not included in extra_sources for posix). Native: link system
    // libs that the manifest doesn't model yet (per-target).
    if !is_embedded_target(&target) && !use_threadx {
        let zenoh_pico_include = if use_system {
            use_system_zenoh_pico()
        } else {
            // Native zenoh-pico include dir for the shim. The
            // unified consumer compiled the static archive; shim
            // build below pulls public headers.
            zenoh_pico_src.join("include")
        };
        if !use_system {
            if target.contains("linux") {
                println!("cargo:rustc-link-lib=pthread");
            } else if target.contains("windows") {
                println!("cargo:rustc-link-lib=ws2_32");
            }
        }
        if backend_count > 0 && !use_zephyr && !use_freertos && !use_nuttx && !use_threadx {
            build_c_shim(
                &c_dir,
                &include_dir,
                &zenoh_pico_include,
                &out_dir,
                use_posix,
                use_bare_metal,
                &target,
                &link_features,
                &shim_config,
            );
        }
    }
    // For Zephyr: C code is built by Zephyr's build system, not Cargo.
    // For no-backend: nothing to build (minimal configuration for header generation).

    // Set cfg flags for Rust code
    if use_posix {
        println!("cargo:rustc-cfg=zpico_backend=\"posix\"");
    } else if use_zephyr {
        println!("cargo:rustc-cfg=zpico_backend=\"zephyr\"");
    } else if use_bare_metal {
        println!("cargo:rustc-cfg=zpico_backend=\"bare-metal\"");
    } else if use_freertos {
        println!("cargo:rustc-cfg=zpico_backend=\"freertos\"");
    } else if use_nuttx {
        println!("cargo:rustc-cfg=zpico_backend=\"nuttx\"");
    } else if use_threadx {
        println!("cargo:rustc-cfg=zpico_backend=\"threadx\"");
    }

    // Phase 160 — probe vendor `_z_sys_net_*_t` sizes BEFORE the alias
    // TU compile so the resulting `net_type_sizes.txt` can be read at
    // alias-TU configure time. Moved earlier than the historical
    // location (post-Rust-cfg block below) so the
    // `_Static_assert(sizeof(nros_zp_alias_*_t) == VENDOR_SIZE)` drift
    // guards inside `platform_aliases.c` actually evaluate; without
    // this reorder the file doesn't exist when alias_build runs the
    // first time after a clean and `#if defined(...)` silently
    // suppresses the assert.
    if backend_count > 0 && !use_zephyr {
        probe_net_type_sizes(
            &c_dir,
            &zenoh_pico_src.join("include"),
            &out_dir,
            use_bare_metal,
            use_freertos,
            use_nuttx,
            use_threadx,
        );
    }

    // Phase 128.D.3 — opt-in alias TU that maps z_*/_z_get_time_*
    // symbols to the canonical nros_platform_* ABI. Compiled only
    // when the `platform-aliases` feature is selected; downstream
    // pairs it with disabling the matching symbols in
    // zpico-platform-shim or relies on `--allow-multiple-definition`
    // for one-cycle co-existence.
    // Phase 154 — FreeRTOS pulls vendor `system/freertos/system.c` +
    // `system/freertos/lwip/network.c` which already provide `z_malloc`
    // / `_z_task_*` / `_z_open_tcp` / etc. with the matching small-
    // socket-struct ABI. Compiling the alias TU on top would emit
    // duplicate symbols at link. Skip on FreeRTOS — vendor src is
    // the single source of truth there.
    //
    // Phase 214.G — gate alias TU emission on an *explicit* platform
    // feature. The TU emits ~30 `_z_*` forwarders that reference
    // canonical `nros_platform_*` symbols (`nros_platform_mutex_*`,
    // `nros_platform_condvar_*`, `nros_platform_time_*`, …). Those
    // symbols come from a paired provider crate that the consumer
    // pulls when it selects a platform — `nros-platform-cffi`'s
    // `posix-c-port` for POSIX hosts, RTOS-specific equivalents
    // elsewhere. Without an explicit platform feature, no provider
    // is guaranteed to be on the link line and the alias TU lands a
    // wall of `undefined symbol: nros_platform_*` errors at every
    // workspace test binary that pulls `zpico-sys` transitively
    // (the `cargo test --workspace` link failure that motivated this
    // gate). Auto-posix (the `target_os = "linux" | "macos" | …`
    // path above) is a build-script convenience for `cargo check` /
    // `cargo build` of `zpico-sys` itself — it does NOT imply the
    // downstream test target enabled `nros-platform/platform-posix`,
    // so it must NOT trigger alias-TU emission. Consumers that want
    // the alias TU keep enabling a platform feature on
    // `nros-rmw-zenoh` / `zpico-sys` (the existing contract); their
    // dep tree carries `nros-platform-cffi/posix-c-port` to satisfy
    // the forwarders. Standalone `cargo test -p zpico-sys` and
    // workspace `--workspace` builds without an explicit feature
    // now get a header-only `zpico-sys` rlib that links anywhere
    // (the resulting rlib must not be loaded at runtime — that's
    // already the same contract the no-backend-selected path emits
    // above).
    // phase-230 1c + Wave 2 (RFC-0034) — FreeRTOS scalar-only alias TU. The
    // full alias path below is gated `!use_freertos` because its net + task +
    // mutex/condvar sections collide with FreeRTOS's vendored
    // `system/freertos/*` primitives (the reason FreeRTOS was excluded). The
    // scalar-only mode emits the `z_malloc`/`z_realloc`/`z_free` (1c) +
    // `z_sleep_*` + `z_random_*` (Wave 2) forwarders → `nros_platform_*`; the
    // vendored copies are guarded out by
    // `Z_FEATURE_NROS_PLATFORM_{ALLOC,SLEEP,RANDOM}` (Step 6.5 on the
    // zenoh-pico build), so exactly these land on the link. Clock/time + the
    // opaque services stay vendored. Same `CARGO_FEATURE_PLATFORM_ALIASES`
    // opt-in keeps the guard + alias coupled.
    if env::var_os("CARGO_FEATURE_PLATFORM_ALIASES").is_some() && use_freertos && platform_resolved
    {
        let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
        let nros_platform_cffi_include = nros_build_paths::nros_platform_cffi_include();
        let mut alias_build = cc::Build::new();
        // issue 0383 — implicit-function-declaration / int-conversion as errors.
        nros_cc_flags::strict_decls(&mut alias_build);
        alias_build
            .file(manifest_dir.join("c/zpico/platform_aliases.c"))
            .include(&nros_platform_cffi_include)
            .include(manifest_dir.join("c/zpico"))
            .define("NROS_ZP_ALIAS_SCALAR_ONLY", None)
            .warnings(true);
        let target_os_for_alias = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
        if target_os_for_alias == "none" {
            alias_build.flag("-ffreestanding");
        }
        // Match zpico.c's `[arch.*]` float-ABI / march / mabi so
        // platform_aliases.o lands in the same archive without an ABI clash.
        if let Some(name) = platform_name
            && let Ok(resolved) = platform_manifest.for_platform(name)
        {
            for arch_name in &resolved.arch {
                if let Some(arch) = platform_manifest.arch.get(arch_name.as_str())
                    && arch_matches(arch, &target)
                {
                    apply_arch(arch, &mut alias_build, &out_dir);
                    break;
                }
            }
        }
        alias_build.compile("zpico_platform_aliases");
        println!("cargo:rerun-if-changed=c/zpico/platform_aliases.c");
        println!("cargo:rerun-if-changed=c/zpico/nros_zenoh_generic_platform.h");
    }

    if env::var_os("CARGO_FEATURE_PLATFORM_ALIASES").is_some() && !use_freertos && platform_resolved
    {
        let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
        let nros_platform_cffi_include = nros_build_paths::nros_platform_cffi_include();
        let mut alias_build = cc::Build::new();
        // issue 0383 — implicit-function-declaration / int-conversion as errors.
        nros_cc_flags::strict_decls(&mut alias_build);
        alias_build
            .file(manifest_dir.join("c/zpico/platform_aliases.c"))
            .include(&nros_platform_cffi_include)
            .include(manifest_dir.join("c/zpico"))
            // Phase 129.D — `NROS_PLATFORM_ALIASES` unlocks the
            // alias TU's clock-variant + network wrappers, which
            // depend on the generic `z_clock_t = uint64_t` typedef
            // and the canonical `_z_sys_net_*` opaque layouts in
            // `nros_zenoh_generic_platform.h`.
            .define("NROS_PLATFORM_ALIASES", None)
            .warnings(true);
        // Phase 160 (ESP32 talker fix, supersedes Phase 156 / 159 /
        // 160.C `USES_UNIX` skip-list) — alias TU's network section is
        // only safe to emit on bare-metal. Every other platform pulls a
        // vendor `system/<rtos>/network.c` into `extra_sources` that
        // provides `_z_open_tcp` / `_z_send_tcp` / etc. with the
        // per-platform `_z_sys_net_{socket,endpoint}_t` layout (4-byte
        // `int _fd` on unix.h, struct embedding `TX_THREAD *` on
        // threadx, etc.). The alias TU's generic 16/32-byte opaque
        // layouts have a DIFFERENT pass-by-value ABI on RV32 (hidden
        // pointer vs. inline registers), so even when symbol resolution
        // picks the vendor copy at link time, any cross-call into the
        // alias TU breaks the calling convention. Worst case (ESP32-C3
        // bare-metal): vendor TX (`link/unicast/tcp.c`) compiles
        // against bare-metal/platform.h's 6-byte endpoint and passes
        // it inline in a1/a2; alias TU's `_z_open_tcp` tail-calls
        // `nros_platform_tcp_open` treating a1 as a pointer →
        // `lhu a2, 2(a1)` faults with a1 = 10.0.2.2 (the IP value).
        // Same class of bug closed NuttX (`nros_support_init -> -4`)
        // in Phase 159. Emit the network section ONLY for bare-metal;
        // every other platform's vendor network.c is the single source
        // of truth and the alias TU stays out of the link.
        if use_threadx {
            // Phase 160 follow-up — threadx uses NROS_PLATFORM_ALIASES
            // (vendor sees the 16/32-byte opaque struct from
            // `nros_zenoh_generic_platform.h`); alias TU emits its
            // network section with the same opaque shape so the
            // by-value pass uses hidden-pointer ABI consistently on
            // both sides. POSIX/NuttX/Zephyr/FreeRTOS bring their own
            // vendor `system/<rtos>/network.c` and stay out of both
            // gates; bare-metal uses the small-struct gate below.
            alias_build.define("NROS_ZP_ALIAS_OPAQUE_NET", None);
        }
        if use_bare_metal {
            alias_build.define("NROS_ZP_ALIAS_BARE_METAL_NET", None);

            // Phase 160 — feed vendor-side `_z_sys_net_socket_t` /
            // `_z_sys_net_endpoint_t` sizes (extracted by `size_probe.c`
            // against the vendor `bare-metal/platform.h`) into the alias
            // TU as preprocessor constants so a `_Static_assert` inside
            // `platform_aliases.c` traps any silent ABI drift between
            // the alias TU's local typedefs and the vendor's. The
            // probe writes `net_type_sizes.txt` (line 985 above) on
            // every bare-metal build; missing file means the probe
            // fell into the warning fallback and we deliberately omit
            // the defines so the static assert is skipped (the
            // fallback already screams).
            let sizes_file = out_dir.join("net_type_sizes.txt");
            if let Ok(contents) = std::fs::read_to_string(&sizes_file) {
                let mut lines = contents.lines();
                let (socket_size, endpoint_size) = (
                    lines.next().and_then(|s| s.trim().parse::<usize>().ok()),
                    lines.next().and_then(|s| s.trim().parse::<usize>().ok()),
                );
                if let (Some(ss), Some(es)) = (socket_size, endpoint_size) {
                    alias_build
                        .define("NROS_ZP_VENDOR_NET_SOCKET_SIZE", ss.to_string().as_str())
                        .define("NROS_ZP_VENDOR_NET_ENDPOINT_SIZE", es.to_string().as_str());
                }
            }
        }
        // Phase 146.1 — ThreadX's `c/platform/threadx/task.c`
        // already provides every `_z_task_*` symbol because the
        // `_z_task_t` layout embeds a `TX_THREAD` struct. Skip the
        // generic alias-TU versions so both TUs can land in the
        // same archive without a duplicate-symbol link error.
        if use_threadx {
            alias_build.define("NROS_PLATFORM_ALIASES_SKIP_TASK", None);
        }
        // Phase 129.D — bare-metal cross targets
        // (`target_os = "none"`) often lack a usable newlib on the
        // host (`#include <stdint.h>` falls into gcc's own header
        // which does `#include_next` expecting newlib).
        // `-ffreestanding` tells gcc to use its own freestanding
        // `<stdint.h>` / `<stddef.h>`, which is all the alias TU
        // actually needs.
        let target_os_for_alias = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
        if target_os_for_alias == "none" {
            alias_build.flag("-ffreestanding");
        }
        // Apply the same `[arch.*]` cflags the manifest hands to
        // zpico.c, so platform_aliases.o uses the matching float-ABI
        // / march / mabi / mcmodel. Without this, riscv64gc targets
        // (ThreadX-RV64) hit `cannot link object files with different
        // floating-point ABI` — rustc emits lp64d while cc-rs picks
        // the bare-metal toolchain default (lp64).
        if let Some(name) = platform_name
            && let Ok(resolved) = platform_manifest.for_platform(name)
        {
            for arch_name in &resolved.arch {
                if let Some(arch) = platform_manifest.arch.get(arch_name.as_str())
                    && arch_matches(arch, &target)
                {
                    apply_arch(arch, &mut alias_build, &out_dir);
                    break;
                }
            }
        }
        alias_build.compile("zpico_platform_aliases");
        println!("cargo:rerun-if-changed=c/zpico/platform_aliases.c");
        println!("cargo:rerun-if-changed=c/zpico/nros_zenoh_generic_platform.h");
    }

    // Rerun triggers
    println!("cargo:rerun-if-changed=c/zpico/zpico.c");
    println!("cargo:rerun-if-changed=c/zpico/nuttx_clock.c");
    println!("cargo:rerun-if-changed=c/platform/bare-metal/platform.h");
    println!("cargo:rerun-if-changed=c/platform/errno_override.h");
    // c/platform/zenoh_generic_config.h was a stale pre-134.3 copy that could
    // shadow the OUT_DIR-generated header on the bare-metal shim include path;
    // deleted in the #135 fix (every TU now consumes the generated config).
    println!("cargo:rerun-if-changed=c/platform/zenoh_generic_platform.h");
    // Watch the TREES, not a list of files (issue 0911).
    //
    // This used to name five zenoh-pico sources out of the several hundred the
    // build actually compiles. Everything else — protocol, transport, codec,
    // collections, the link implementations — had no rebuild edge, so patching
    // one of them changed nothing: cargo saw no watched input move, skipped the
    // build script, and the test ran against a binary without the edit. That
    // does not fail; it produces a green run whose conclusion is about code that
    // was never compiled. It cost a full measure-fix-measure cycle in issue 0899
    // and nearly produced a "the fix does not work" verdict about a fix that had
    // never been built.
    //
    // `cargo:rerun-if-changed` accepts a directory and cargo walks it, so two
    // lines cover the compiled set with no list to maintain. The cost is that any
    // touch under these trees re-runs the build script; for a vendored library
    // that changes only when someone patches it, that is the right trade.
    println!("cargo:rerun-if-changed=zenoh-pico/src");
    println!("cargo:rerun-if-changed=zenoh-pico/include");
    println!("cargo:rerun-if-changed=c/zenoh-pico-version.h.in");
    println!("cargo:rerun-if-changed=zenoh-pico/version.txt");
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");
    // issue 0491 — `FREERTOS_DIR` / `LWIP_DIR` / `FREERTOS_CONFIG_DIR` /
    // `NUTTX_DIR` name DIRECTORIES and are not fingerprinted as strings; the
    // headers this shim compiles against are declared per file above. The PORT
    // is a name, so it stays.
    println!("cargo:rerun-if-env-changed=FREERTOS_PORT");
    println!("cargo:rerun-if-changed=c/size_probe.c");

    // Phase 160 — probe moved to before alias-TU compile (see
    // comment above `probe_net_type_sizes` call earlier in this
    // function).
}

/// Probe the sizes of `_z_sys_net_socket_t` and `_z_sys_net_endpoint_t` from C headers.
///
/// Compiles `c/size_probe.c` with the same platform defines as zenoh-pico,
/// reads the symbol sizes from the resulting .o file, and emits them as
/// `cargo:SOCKET_SIZE=<N>` and `cargo:ENDPOINT_SIZE=<N>` DEP variables.
/// zpico-platform-shim reads these as `DEP_ZPICO_SOCKET_SIZE` / `DEP_ZPICO_ENDPOINT_SIZE`.
#[allow(clippy::too_many_arguments)]
fn probe_net_type_sizes(
    c_dir: &Path,
    zenoh_pico_include: &Path,
    out_dir: &Path,
    use_bare_metal: bool,
    use_freertos: bool,
    use_nuttx: bool,
    use_threadx: bool,
) {
    let mut build = cc::Build::new();
    // issue 0383 — implicit-function-declaration / int-conversion as errors.
    nros_cc_flags::strict_decls(&mut build);
    build.file(c_dir.join("size_probe.c"));
    build.include(zenoh_pico_include);
    // Issue #135 — the ZENOH_GENERIC branches below used to find
    // <zenoh_generic_config.h> via the stale c/platform copy (deleted);
    // give every probe the OUT_DIR-generated config the library uses.
    build.include(out_dir.join("zenoh-config"));
    // Issue 0626 — the platform headers now name `nros_platform_task_attr_t`
    // for `z_task_attr_t`, so the probe needs `<nros/platform.h>` like every
    // other TU that includes them. The probe measures the NET types, but it
    // reaches them through the same platform header, so its include set has to
    // match — it failed with `nros/platform.h: No such file or directory` on
    // the ThreadX cross build until this was added.
    build.include(nros_build_paths::nros_platform_cffi_include());

    // Set the same platform defines as the main build so platform.h selects
    // the correct platform header (unix.h, freertos/lwip.h, void.h, etc.)
    if use_bare_metal {
        // bare-metal: ZENOH_GENERIC → zenoh_generic_platform.h → bare-metal/platform.h
        build.define("ZENOH_GENERIC", None);
        let platform_dir = c_dir.join("platform");
        build.include(&platform_dir);

        // RV32 bare-metal (ESP32-C3): mirror `build_zenoh_pico_embedded` —
        // add the cross-compile flags, errno override and picolibc sysroot
        // so the probe finds <stdint.h>. Without this the probe emits
        // `fatal error: stdint.h: No such file or directory` (picolibc
        // headers aren't on the default system include path for
        // `riscv64-unknown-elf-gcc -march=rv32imc`) and falls back to the
        // hardcoded 16/8 defaults.
        let target = env::var("TARGET").unwrap_or_default();
        if target.contains("riscv32") {
            detect_riscv_compiler(&mut build);
            build.flag("-march=rv32imc").flag("-mabi=ilp32");

            let errno_dir = out_dir.join("errno-override");
            std::fs::create_dir_all(&errno_dir).ok();
            std::fs::write(
                errno_dir.join("errno.h"),
                include_bytes!("../../zpico-sys/c/platform/errno_override.h"),
            )
            .ok();
            build.include(&errno_dir);

            if let Some(sysroot) = get_picolibc_sysroot() {
                build.include(sysroot.join("include"));
            }
        } else if target.contains("thumbv7m") || target.contains("thumbv7em") {
            // ARM Cortex-M bare-metal: cc crate selects arm-none-eabi-gcc
            // which ships its own newlib sysroot, so no sysroot include
            // is needed here — but the -mcpu flags are required for the
            // preprocessor to pick the right architecture-dependent
            // headers.
            if target.contains("thumbv7em") {
                build
                    .flag("-mcpu=cortex-m4")
                    .flag("-mthumb")
                    .flag("-mfpu=fpv4-sp-d16")
                    .flag("-mfloat-abi=hard");
            } else {
                build.flag("-mcpu=cortex-m3").flag("-mthumb");
            }
        } else if target.contains("armv7r") {
            // Phase 100.6 — AGX Orin SPE (Cortex-R5F). cc emits
            // `-march=armv7-r` from the target triple; we add
            // hard-float matching the FSP's vfpv3-d16 build. Same
            // flags as `zpico-platform-shim/build.rs`'s armv7r branch.
            build.flag("-mfpu=vfpv3-d16").flag("-mfloat-abi=hard");
        }
    } else if use_freertos {
        build.define("ZENOH_FREERTOS_LWIP", None);
        // lwIP + FreeRTOS headers needed
        if let Ok(dir) = env::var("FREERTOS_DIR") {
            build.include(PathBuf::from(&dir).join("include"));
            if let Ok(port) = env::var("FREERTOS_PORT") {
                build.include(PathBuf::from(&dir).join("portable").join(&port));
            }
        }
        if let Ok(dir) = env::var("FREERTOS_CONFIG_DIR") {
            build.include(dir);
        }
        if let Ok(dir) = env::var("LWIP_DIR") {
            let lwip = PathBuf::from(dir);
            build.include(lwip.join("src/include"));
        }
    } else if use_nuttx {
        build.define("ZENOH_NUTTX", None);
        // Issue 1039 — this used to be `ZENOH_LINUX`, defined ALONGSIDE
        // ZENOH_NUTTX so that the six `#if defined(ZENOH_LINUX)` arms in
        // zenoh-pico's `src/system/unix/system.c` would compile: NuttX ships
        // `<sys/random.h>` and `getrandom()`, and the NuttX arm they shadow
        // opens `/dev/urandom`, a device node a NuttX configuration is not
        // obliged to provide.
        //
        // The arm was right and the NAME was wrong. Reaching a capability by
        // asserting a platform makes the image claim to BE Linux for every
        // OTHER guard that tests the same macro, which is why dropping the
        // define was not a one-line change: three guards in
        // `system/common/platform.c` (`<arpa/inet.h>` and the two endpoint
        // helpers) were being reached through it too, and NuttX has no
        // endpoint implementation of its own to fall back to.
        //
        // Those guards now name NuttX, the randomness arms test
        // `ZENOH_HAS_GETRANDOM`, and this says what the port actually has.
        // Proven equivalent, not assumed: `gcc -E -P` output for
        // `system/unix/system.c`, `system/common/platform.c` and
        // `system/unix/network.c` is byte-identical before and after, so every
        // NuttX image compiles the code it compiled before.
        //
        // Still worth knowing: the six `ZENOH_NUTTX` randomness arms remain
        // DEAD in our builds, because we do define the capability. They are
        // now REACHABLE — a NuttX port without `getrandom()` selects them by
        // omitting this define — which they were not before.
        build.define("ZENOH_HAS_GETRANDOM", None);
        // Issue 0551 — the SHARED tree's `include/` is not a guaranteed compile
        // input. `build-nuttx.sh`'s snapshot short-circuit says so in as many
        // words ("this path guarantees the SNAPSHOT, never the tree"): when the
        // key matches it skips the build entirely and leaves `$NUTTX_DIR`
        // holding whichever arch was configured LAST — or, after any
        // `make olddefconfig`, holding no generated `nuttx/config.h` at all,
        // since that target runs `clean_context` which deletes it.
        //
        // That is issue 0525's rule, and this was its sixth site: the gate
        // greps for a receiver NAMED nuttx, and here the binding is `dir`.
        if let Ok(dir) = env::var("NUTTX_DIR") {
            build.include(nros_build_paths::nuttx_include_root(&PathBuf::from(dir)));
        }
    } else if use_threadx {
        build.define("ZENOH_GENERIC", None);
        build.define("ZENOH_THREADX", None);
        let platform_dir = c_dir.join("platform");
        build.include(&platform_dir);

        // ThreadX platform.h includes `tx_api.h` which pulls in ThreadX
        // kernel headers + a port-specific `tx_port.h`. Without these
        // the probe compile fails and we fall back to the hardcoded
        // 16/8 default — which silently skews the pass-by-value ABI
        // of `_z_sys_net_socket_t` at the FFI boundary (the Rust
        // shim ends up reading garbage from the wrong registers).
        //
        // Mirror the include set the main build uses for ThreadX.
        let target = env::var("TARGET").unwrap_or_default();
        if let Ok(dir) = env::var("THREADX_DIR") {
            let threadx_dir = PathBuf::from(&dir);
            build.include(threadx_dir.join("common/inc"));
            // Pick the port-specific header matching the target arch.
            if target.contains("riscv64") {
                build.include(threadx_dir.join("ports/risc-v64/gnu/inc"));
            } else if !is_embedded_target(&target) {
                build.include(threadx_dir.join("ports/linux/gnu/inc"));
            }
        }
        if let Ok(dir) = env::var("THREADX_CONFIG_DIR") {
            build.include(dir);
        }
        if let Ok(dir) = env::var("NETX_DIR") {
            let netx_dir = PathBuf::from(&dir);
            build.include(netx_dir.join("common/inc"));
            build.include(netx_dir.join("addons/BSD"));
            if !is_embedded_target(&target) {
                build.include(netx_dir.join("ports/linux/gnu/inc"));
            }
        }
        if let Ok(dir) = env::var("NETX_CONFIG_DIR") {
            build.include(dir);
        }

        // Cross-compile flags + C-library sysroot for bare-metal
        // RISC-V. Without these the probe fails with
        // `stdint.h: No such file or directory` and falls back to
        // the hardcoded default sizes.
        if target.contains("riscv64") {
            build
                .flag("-march=rv64gc")
                .flag("-mabi=lp64d")
                .flag("-mcmodel=medany");

            // errno-override header (picolibc's TLS errno doesn't
            // work on bare-metal). Must be searched before picolibc.
            let errno_dir = out_dir.join("errno-override");
            std::fs::create_dir_all(&errno_dir).ok();
            std::fs::write(
                errno_dir.join("errno.h"),
                include_bytes!("../../zpico-sys/c/platform/errno_override.h"),
            )
            .ok();
            build.include(&errno_dir);

            if let Some(sysroot) = get_picolibc_sysroot() {
                build.include(sysroot.join("include"));
            }
        }
    } else {
        // POSIX: zenoh-pico auto-detects ZENOH_LINUX/ZENOH_MACOS from target
        let target = env::var("TARGET").unwrap_or_default();
        if target.contains("linux") {
            build.define("ZENOH_LINUX", None);
        }
    }

    // Generated config header (for Z_FEATURE_LINK_TCP etc.)
    let generated_config_dir = out_dir.join("zenoh-config");
    if generated_config_dir.exists() {
        build.include(&generated_config_dir);
    }

    // Compile to a separate static library (may fail on targets without
    // C standard library headers, e.g. RISC-V without picolibc)
    build.cargo_metadata(false); // Don't emit link flags
    if let Err(e) = build.try_compile("size_probe") {
        // Probe failure means the `_z_sys_net_socket_t` /
        // `_z_sys_net_endpoint_t` sizes are UNKNOWN. Guessing them skews the
        // pass-by-value FFI ABI (the Rust shim reads its opaque buffer from
        // the wrong call-site registers) and the runtime failure mode is a
        // silent `Transport(ConnectionFailed)` at session open (zero-length
        // send, no read) — the issue-0135 mismatched-TU class, reintroduced
        // dynamically. Issue #207: on CROSS targets there is no defensible
        // guess, so fail the build loudly instead of shipping a corrupt ABI.
        //
        // To diagnose: rerun `cargo build` with `-vv` to see the underlying
        // `cc::try_compile` error, and add the missing include path to
        // `probe_net_type_sizes` for that backend.
        let target = env::var("TARGET").unwrap_or_default();
        let host = env::var("HOST").unwrap_or_default();
        if target != host {
            panic!(
                "zpico-sys size_probe failed for cross target {target} ({e}); \
                 the _z_sys_net_socket_t/_z_sys_net_endpoint_t sizes cannot be \
                 guessed — a wrong size silently corrupts the pass-by-value \
                 FFI ABI (Transport(ConnectionFailed) at session open). \
                 Rerun with `cargo build -vv` for the compiler error and fix \
                 the probe's include paths for this backend."
            );
        }
        // Host-native fallback only: the historical defaults match the
        // glibc/macOS layouts the probe was derived from; keep the warning
        // so a host layout change is still visible.
        println!(
            "cargo:warning=zpico-sys size_probe failed on host target ({e}); \
             falling back to SOCKET_SIZE=16 / ENDPOINT_SIZE=8"
        );
        println!("cargo:SOCKET_SIZE=16");
        println!("cargo:ENDPOINT_SIZE=8");
        return;
    }

    // Read symbol sizes from the compiled archive using llvm-nm.
    // The probe C file defines arrays whose lengths equal sizeof(type).
    let archive = out_dir.join("libsize_probe.a");
    let rustc_sysroot = env::var("RUSTC_SYSROOT").ok();
    let host = env::var("HOST").unwrap_or_default();
    let socket_size = read_symbol_size(
        &archive,
        "__nros_sizeof_net_socket",
        rustc_sysroot.as_deref(),
        &host,
    );
    let endpoint_size = read_symbol_size(
        &archive,
        "__nros_sizeof_net_endpoint",
        rustc_sysroot.as_deref(),
        &host,
    );

    // Emit as DEP variables (available to direct dependent crates as DEP_ZPICO_*)
    println!("cargo:SOCKET_SIZE={}", socket_size);
    println!("cargo:ENDPOINT_SIZE={}", endpoint_size);

    // Also emit as rustc-env so zpico-platform-shim can read them.
    // zpico-platform-shim is a dependency of zpico-sys (not the other way),
    // so DEP variables don't flow. Instead, write a shared file.
    let sizes_file = out_dir.join("net_type_sizes.txt");
    std::fs::write(&sizes_file, format!("{}\n{}\n", socket_size, endpoint_size)).unwrap();
    // Export the path so zpico-platform-shim's build.rs can find it
    println!(
        "cargo:rustc-env=ZPICO_NET_SIZES_FILE={}",
        sizes_file.display()
    );
}

/// Compare the committed `zpico.h` against a fresh cbindgen render and warn on
/// drift.
///
/// phase-400 W2a — the whole body is behind `cbindgen-drift-check` and off by
/// default. Since issue 0452 this writes nothing, so with the feature off the
/// build simply uses the committed header, exactly as it did with the feature
/// on; what is lost is the early warning, not the check
/// (`check-cbindgen-headers` is unconditional and hard-fails).
#[cfg(feature = "cbindgen-drift-check")]
fn generate_header(manifest_dir: &Path, include_dir: &Path) {
    // Create include directory if needed
    if !include_dir.exists() {
        std::fs::create_dir_all(include_dir).unwrap_or_else(|e| {
            println!("cargo:warning=Failed to create include directory: {e}");
        });
    }

    let output_file = include_dir.join("zpico.h");
    let config_file = manifest_dir.join("cbindgen.toml");

    // Load cbindgen config
    let config = cbindgen::Config::from_file(&config_file).unwrap_or_else(|e| {
        println!("cargo:warning=Failed to load cbindgen config: {e}");
        cbindgen::Config::default()
    });

    // Generate header
    let mut header = Vec::new();
    let result = cbindgen::Builder::new()
        .with_crate(manifest_dir)
        .with_config(config)
        .generate();

    match result {
        Ok(bindings) => {
            bindings.write(&mut header);
            let header_str = String::from_utf8_lossy(&header);

            // Post-process: remove lines starting with "extern " and collapse blank lines
            let processed = crate::post_process_header(&header_str);

            if !crate::is_plausible_generated_header(&processed) {
                println!(
                    "cargo:warning=cbindgen generated an incomplete zpico.h; keeping existing header"
                );
                return;
            }

            // Issue 0452 — this header is COMMITTED, and a build must not write
            // the source tree. It used to: a per-pid temp plus an atomic rename,
            // which made ONE writer safe but still meant every build could dirty
            // the worktree when the graph resolved a different cbindgen patch
            // release. Now the build only COMPARES; `just regen-c-headers` is
            // the single writer, and `check-cbindgen-headers` is the gate.
            //
            // The whole reason the race machinery existed — N parallel cargo
            // invocations, one per example/target-dir, all rebuilding zpico-sys
            // against this same source-tree path — is gone with the write.
            let existing = std::fs::read(&output_file).ok();
            if existing.as_deref() != Some(processed.as_bytes()) {
                println!(
                    "cargo:warning={} is STALE against this crate's sources — run \
                     `just regen-c-headers` and commit the result (issue 0452). \
                     The build used the committed copy.",
                    output_file.display()
                );
            }
        }
        Err(e) => {
            println!("cargo:warning=cbindgen failed: {e}");
        }
    }
}

/// Use a pre-built zenoh-pico from `ZENOH_PICO_DIR` (system-zenohpico
/// feature). Expects a CMake install prefix layout:
///   $ZENOH_PICO_DIR/lib/libzenohpico.a
///   $ZENOH_PICO_DIR/include/zenoh-pico.h
fn use_system_zenoh_pico() -> PathBuf {
    let zenoh_pico_dir = env::var("ZENOH_PICO_DIR").unwrap_or_else(|_| {
        panic!(
            "ZENOH_PICO_DIR environment variable is required when system-zenohpico feature is enabled.\n\
             Set it to the CMake install prefix of your zenoh-pico build, e.g.:\n\
             ZENOH_PICO_DIR=/path/to/zenoh-pico-install cargo build --features system-zenohpico"
        );
    });
    // `ZENOH_PICO_DIR` is a PATH, so its SPELLING is not fingerprinted
    // (issue 0491); the two files the prefix is actually consumed as are
    // watched instead.
    let dir = PathBuf::from(&zenoh_pico_dir);
    let lib_path = dir.join("lib").join("libzenohpico.a");
    let header_path = dir.join("include").join("zenoh-pico.h");
    println!("cargo:rerun-if-changed={}", lib_path.display());
    println!("cargo:rerun-if-changed={}", header_path.display());
    if !lib_path.exists() {
        panic!(
            "ZENOH_PICO_DIR={}: expected static library at {}\n\
             Build zenoh-pico with: cmake --build <build> && cmake --install <build>",
            zenoh_pico_dir,
            lib_path.display()
        );
    }
    if !header_path.exists() {
        panic!(
            "ZENOH_PICO_DIR={}: expected version header at {}\n\
             Build zenoh-pico with: cmake --build <build> && cmake --install <build>",
            zenoh_pico_dir,
            header_path.display()
        );
    }
    println!(
        "cargo:rustc-link-search=native={}",
        dir.join("lib").display()
    );
    println!("cargo:rustc-link-lib=static=zenohpico");
    let target = env::var("TARGET").unwrap_or_default();
    if target.contains("linux") {
        println!("cargo:rustc-link-lib=pthread");
    } else if target.contains("windows") {
        println!("cargo:rustc-link-lib=ws2_32");
    }
    println!(
        "cargo:warning=Using system zenoh-pico from {}. \
         Ensure it was built with compatible Z_FEATURE_* flags \
         (Z_FEATURE_INTEREST=1, Z_FEATURE_MATCHING=1).",
        zenoh_pico_dir
    );
    dir.join("include")
}

/// Build the C shim library
///
/// issue 0899 / 0902 — opt-in DWARF for the vendored C, `NROS_ZPICO_DEBUG=1`.
///
/// cc-rs takes `-g` from cargo's `DEBUG`, which comes from the profile — and the
/// C/C++ example path builds through corrosion, which pins its own profile, so
/// neither `NROS_CARGO_PROFILE` nor `CMAKE_BUILD_TYPE` nor a `debug =` on a
/// carve-out reaches it (all three measured: the build still reports
/// `profile [optimized]`). The result was a debugger that could name
/// `_z_wbuf_put` and nothing inside it — no line numbers, no `ptype`.
///
/// This is deliberately an ENV knob rather than a profile change: it is a
/// debugging session's tool, and turning it on by default would put DWARF for a
/// vendored library into every embedded image that links it.
fn zpico_debug_requested() -> bool {
    println!("cargo:rerun-if-env-changed=NROS_ZPICO_DEBUG");
    matches!(
        std::env::var("NROS_ZPICO_DEBUG").as_deref(),
        Ok("1") | Ok("true")
    )
}

/// Note: For Zephyr, C code is built by Zephyr's build system, not here.
#[allow(clippy::too_many_arguments)]
fn build_c_shim(
    c_dir: &Path,
    include_dir: &Path,
    zenoh_pico_include: &Path,
    out_dir: &Path,
    use_posix: bool,
    use_bare_metal: bool,
    target: &str,
    link: &LinkFeatures,
    shim: &ShimConfig,
) {
    let mut build = cc::Build::new();
    // issue 0383 — implicit-function-declaration / int-conversion as errors.
    nros_cc_flags::strict_decls(&mut build);

    // Include paths
    //
    // Issue #135 — the generated zenoh config MUST come first and the shim
    // MUST compile with `ZENOH_GENERIC`, exactly like the zenoh-pico library
    // TUs built by `build_zenoh_pico_unified`. `z_get_options_t` (and other
    // public structs) change LAYOUT with `Z_FEATURE_LOCAL_QUERYABLE` /
    // `Z_FEATURE_LOCAL_SUBSCRIBER`; a shim TU falling back to the in-tree
    // `zenoh-pico/config.h` defaults while the library uses the generated
    // header is an ABI mismatch: the library's `z_get` read the shim's
    // `opts.target` (`Z_QUERY_TARGET_ALL` = 1) as `opts.allowed_destination`
    // (`Z_LOCALITY_SESSION_LOCAL` = 1), so every cross-process query was
    // silently downgraded to session-local and never reached the router.
    build.include(out_dir.join("zenoh-config"));
    build.define("ZENOH_GENERIC", None);
    build.include(include_dir);
    build.include(zenoh_pico_include);
    // Phase 154 — `zpico.c` now `#include <nros/platform_net.h>` from
    // `nros-platform-cffi`. The unified (embedded) builder picks the
    // path up via the manifest's `include_paths`; the legacy
    // `build_c_shim` path (POSIX + bare-metal) still needs it added
    // explicitly so `cargo check --workspace` on the host doesn't
    // fail with `nros/platform_net.h: No such file or directory`.
    build.include(nros_build_paths::nros_platform_cffi_include());

    // Core shim source
    build.file(c_dir.join("zpico/zpico.c"));

    // Platform-specific configuration
    if use_posix {
        #[cfg(target_os = "linux")]
        build.define("ZENOH_LINUX", None);
        // Mirror `posix nros-platform.toml [build.zenoh] defines_kv` — the
        // generated header's `#ifndef Z_FEATURE_MULTI_THREAD` fallback is 0,
        // so without this the shim would flip single-threaded while the
        // library runs multi-threaded (same ABI-divergence class as #135).
        build.define("Z_FEATURE_MULTI_THREAD", "1");
        build.define("ZENOH_DEBUG", "0");
    } else if use_bare_metal {
        let platform_dir = c_dir.join("platform");

        // Include platform headers
        build.include(&platform_dir);

        // Platform defines — link features from Cargo features.
        //
        // Phase 132 — `ZPICO_NO_SMOLTCP=1` lets a consumer (the
        // serial-only board crate, an XRCE-over-UART firmware, …)
        // opt out of compiling the smoltcp glue into `zpico.c`.
        // Phase 128 retired the per-transport Cargo features that
        // used to gate this; without an override the embedded build
        // always pulls smoltcp because `LinkFeatures::from_env()`
        // hardcodes tcp/udp = true. Serial-only firmware links
        // against `ZPICO_SERIAL` instead and never provides the
        // `smoltcp_init` / `smoltcp_cleanup` symbols.
        let opt_out_smoltcp = env::var("ZPICO_NO_SMOLTCP").is_ok();
        println!("cargo:rerun-if-env-changed=ZPICO_NO_SMOLTCP");
        let has_network = (link.tcp || link.udp_unicast || link.udp_multicast) && !opt_out_smoltcp;
        if has_network {
            build.define("ZPICO_SMOLTCP", None);
        }
        if link.serial && !has_network {
            build.define("ZPICO_SERIAL", None);
        }
        // ZENOH_GENERIC is set unconditionally above (issue #135).
        build.define("Z_FEATURE_MULTI_THREAD", "0");
        // Phase 134.4 — every `Z_FEATURE_LINK_*` / `Z_FEATURE_RAWETH_*`
        // / `Z_FEATURE_SCOUTING_UDP` / `Z_FEATURE_UNSTABLE_API` value
        // lives in `<out_dir>/zenoh-config/zenoh_generic_config.h`,
        // generated by `generate_config_header` from the resolved
        // `LinkFeatures + LinkPolicy`. The compile units that need
        // them `#include "zenoh-pico/config.h"` which dispatches into
        // our header under `ZENOH_GENERIC`. NO `Z_FEATURE_LINK_*`
        // literals scattered through `build.rs`.
        let _ = link;

        // ARM cross-compilation flags
        if target.contains("thumbv7em") {
            build
                .flag("-mcpu=cortex-m4")
                .flag("-mthumb")
                .flag("-mfpu=fpv4-sp-d16")
                .flag("-mfloat-abi=hard");
        }

        // RISC-V cross-compilation flags (ESP32-C3)
        if target.contains("riscv32imc") {
            // issue 0399 — pick the riscv cross-compiler explicitly. Without
            // this, cc-rs derives `riscv32-unknown-elf-gcc` from the triple —
            // a name nothing installs — and the shim build dies on a host
            // provisioned exactly as documented (`nros setup` ships
            // `riscv-none-elf-gcc`). `detect_riscv_compiler` walks
            // `RISCV_GCC_CANDIDATES`, which now includes it. Mirrors the
            // manifest-driven lib path (`apply_arch` → `detect_riscv_compiler`).
            crate::detect_riscv_compiler(&mut build);
            build.flag("-march=rv32imc").flag("-mabi=ilp32");

            // picolibc provides C standard library headers (stdint.h, etc.)
            // for the detected riscv cross toolchain
            if has_picolibc_specs() {
                build.flag("--specs=picolibc.specs");
            }
        }
    }

    // Pass shim slot counts as -D flags so zpico.c gets them
    shim.apply_to_cc(&mut build);

    build.opt_level(2);
    if zpico_debug_requested() {
        build.debug(true);
    }
    build.compile("zpico");
}

/// Phase 136.4 — unified zenoh-pico cc-rs builder, driven by the
/// resolved `[build.zenoh]` block from the platform package's `nros-platform.toml` (RFC-0049).
/// Replaces the five per-RTOS functions (`build_zenoh_pico_{embedded,
/// orin_spe, freertos, nuttx, threadx}` — historical names) and the POSIX
/// `build_zenoh_pico_native` body. Per-platform deltas all come from
/// the manifest: defines, required env vars, include paths
/// (interpolated `{env:VAR}` / `{nros}` / `{src}` / `{out}`),
/// conditional include paths (`when.target_match` /
/// `when.target_not` / `when.if_env`), extra C sources (with
/// `if_env` and `with_define` modifiers), the `[arch.*]` profile
/// (cflags + picolibc / errno-override / riscv-compiler hooks),
/// compile settings, and the `pic` flag.
#[allow(clippy::too_many_arguments)]
fn build_zenoh_pico_unified(
    plat: &manifest::ResolvedPlatform,
    arch_table: &std::collections::BTreeMap<String, manifest::ArchEntry>,
    interp: &manifest::InterpContext<'_>,
    zenoh_pico_src: &Path,
    out_dir: &Path,
    target: &str,
    link: &LinkFeatures,
    shim: &ShimConfig,
    caps: &std::collections::BTreeMap<String, bool>,
) {
    let is_embedded = is_embedded_target(target);
    // Step 1 — validate required env vars (loud panic with help).
    for req in &plat.required_env {
        // issue 1143 — an SDK is required only where this build uses it. A
        // FreeRTOS board with no netstack must not have to invent an
        // `LWIP_DIR` before its build will start.
        if !manifest::matches(&req.when, target, is_embedded, caps) {
            continue;
        }
        let val = env::var(&req.name).unwrap_or_else(|_| {
            panic!("{} not set. {}", req.name, req.help);
        });
        if let Some(subdir) = &req.validate_subdir {
            let path = PathBuf::from(&val).join(subdir);
            if !path.exists() {
                panic!(
                    "{}={}: missing {} (expected at {}). {}",
                    req.name,
                    val,
                    subdir,
                    path.display(),
                    req.help
                );
            }
        }
    }

    let mut build = cc::Build::new();
    // issue 0383 — implicit-function-declaration / int-conversion as errors.
    // This is the zenoh-pico library compile: the TWO calls that made 0383 a
    // real outage (`_z_open_serial_from_*`, `_z_open_custom`) are in this tree.
    nros_cc_flags::strict_decls(&mut build);

    // Step 2 — version header (shared with embedded path).
    let version_include_dir = out_dir.join("zenoh-pico-version");
    generate_embedded_version_header(zenoh_pico_src, &version_include_dir);

    // Step 3 — arch profile (cflags + sysroot / errno-override /
    // riscv-cc probe). Platform's `arch` is now a Vec (Phase 148);
    // walk entries in order and apply the first whose `target_match`
    // hits the build target. Multi-arch platforms (bare-metal across
    // cortex-m3 + riscv32imc) thus map to the right profile per
    // target instead of being hard-coded to one arch.
    for arch_name in &plat.arch {
        let arch = arch_table.get(arch_name.as_str()).unwrap_or_else(|| {
            panic!(
                "zenoh_platforms.toml: platform `{}` references unknown arch `{}`",
                plat.name, arch_name
            )
        });
        if arch_matches(arch, target) {
            apply_arch(arch, &mut build, out_dir);
            break;
        }
    }

    // Step 4 — core sources + per-platform extra C files.
    add_zenoh_pico_core_sources(&mut build, zenoh_pico_src);
    for extra in &plat.extra_sources {
        // issue 1143 — the `when` gate, composed with `if_env` (both must
        // pass). An empty matcher is always true, which is every row that
        // predates it.
        if !manifest::matches(&extra.when, target, is_embedded, caps) {
            continue;
        }
        if let Some(env_var) = &extra.if_env {
            if env::var(env_var).is_err() {
                continue;
            }
            println!("cargo:rerun-if-env-changed={env_var}");
        }
        let path_str = manifest::interpolate(&extra.path, interp).unwrap_or_else(|e| {
            panic!(
                "zenoh_platforms.toml: platform `{}` extra_sources `{}`: {e}",
                plat.name, extra.path
            )
        });
        build.file(&path_str);
        if let Some(def) = &extra.with_define {
            let value = def.get(1).map(|s| s.as_str());
            build.define(&def[0], value);
        }
    }

    // Step 5 — include paths (unconditional + conditional).
    let zenoh_config_dir = out_dir.join("zenoh-config");
    build
        .include(&zenoh_config_dir)
        .include(zenoh_pico_src.join("include"))
        .include(&version_include_dir);
    build.include(nros_build_paths::nros_platform_cffi_include());
    for raw in &plat.include_paths {
        let path = manifest::interpolate(raw, interp).unwrap_or_else(|e| {
            panic!(
                "zenoh_platforms.toml: platform `{}` include_paths `{raw}`: {e}",
                plat.name
            )
        });
        // issue 0491 — these paths interpolate `{env:…}` PATH variables, whose
        // CONTENT is the real build input. Watched here; the variables'
        // spellings are not fingerprinted (a leaf spells one directory once
        // per leaf, `just` spells it absolute, and the two flipped every
        // ThreadX row's zpico-sys build script on every probe).
        nros_build_paths::watch_path(std::path::Path::new(&path));
        build.include(&path);
    }
    for cond in &plat.include_paths_conditional {
        if !manifest::matches(&cond.when, target, is_embedded, caps) {
            continue;
        }
        let path = manifest::interpolate(&cond.path, interp).unwrap_or_else(|e| {
            panic!(
                "zenoh_platforms.toml: platform `{}` conditional include `{}`: {e}",
                plat.name, cond.path
            )
        });
        // issue 0491 — see the unconditional loop above.
        nros_build_paths::watch_path(std::path::Path::new(&path));
        build.include(&path);
    }

    // Step 6 — defines (unconditional, key=value, env-derived).
    //
    // #189 — the Phase-132 serial-only opt-out, restored post-manifest-
    // migration. Phase 136.4 moved the bare-metal compile from
    // `build_c_shim` (which honored `ZPICO_NO_SMOLTCP`) into this manifest
    // path, whose `[platform.bare-metal] defines` hardcode `ZPICO_SMOLTCP`
    // — so every serial-only firmware compiled the smoltcp spin branch,
    // whose clock (`smoltcp_clock_now_ms`) is frozen without a smoltcp
    // iface: `zpico_spin_once(10)` could only return on router traffic
    // (~2.5 s keepalives) and the no_std executor credits just the
    // requested 10 ms per spin, so timers never came due (serial pubsub
    // published 0 forever). With the opt-out set and a serial-only link
    // set, swap in `ZPICO_SERIAL` — the branch built for exactly this.
    let opt_out_smoltcp = env::var("ZPICO_NO_SMOLTCP").is_ok();
    println!("cargo:rerun-if-env-changed=ZPICO_NO_SMOLTCP");
    let serial_only = link.serial && !(link.tcp || link.udp_unicast || link.udp_multicast);
    for define in &plat.defines {
        if define == "ZPICO_SMOLTCP" && opt_out_smoltcp && serial_only {
            build.define("ZPICO_SERIAL", None);
            continue;
        }
        build.define(define, None);
    }
    // issue 1143 — `when`-gated defines. This is how the FreeRTOS family
    // picks its zenoh-pico platform header: `ZENOH_FREERTOS_LWIP` selects
    // `freertos/lwip.h`, whose `_z_sys_net_socket_t` is `{ int _socket; }` and
    // which `#include`s `lwip/sockets.h`; a board with no IP stack takes
    // `ZENOH_ORIN_SPE` and the placeholder socket instead. One define, one
    // socket ABI, every TU in the archive agreeing — the 0135 rule.
    for cond in &plat.defines_conditional {
        if !manifest::matches(&cond.when, target, is_embedded, caps) {
            continue;
        }
        build.define(&cond.name, None);
    }
    for (key, value) in &plat.defines_kv {
        build.define(key, value.as_str());
    }
    for (key, env_def) in &plat.defines_env {
        let value = env::var(&env_def.env).unwrap_or_else(|_| env_def.default.clone());
        build.define(key, value.as_str());
        println!("cargo:rerun-if-env-changed={}", env_def.env);
    }

    // Step 6.5 — phase-230 1c (RFC-0034): scalar-alloc funnel guard.
    // When the consumer pulls the memory-only alias TU (the `platform-aliases`
    // feature) AND this is the FreeRTOS unified build, define
    // `Z_FEATURE_NROS_PLATFORM_ALLOC` so `system/freertos/system.c` drops its
    // vendored `z_malloc`/`z_realloc`/`z_free` (→ `pvPortMalloc`). The
    // alias TU then supplies them as `nros_platform_alloc`/`_realloc`/
    // `_dealloc` forwarders — one heap funnel, one stats counter. Coupled to
    // `CARGO_FEATURE_PLATFORM_ALIASES`: a serial-only node that drops
    // `platform-aliases` (via `default-features = false`) gets neither the
    // guard nor the alias, so the vendored heap stays intact (no undefined
    // `z_malloc`). The matching memory-only alias compile lives in the alias
    // gate below.
    if env::var_os("CARGO_FEATURE_PLATFORM_ALIASES").is_some()
        && env::var_os("CARGO_FEATURE_FREERTOS").is_some()
    {
        build.define("Z_FEATURE_NROS_PLATFORM_ALLOC", None);
        // phase-230 Wave 2 — extend the scalar funnel to sleep + random
        // (clock/time stay vendored: `z_clock_t` is FreeRTOS's `TickType_t`).
        // The scalar alias TU emits the matching forwarders (see the
        // `NROS_ZP_ALIAS_SCALAR_ONLY` compile below).
        build.define("Z_FEATURE_NROS_PLATFORM_SLEEP", None);
        build.define("Z_FEATURE_NROS_PLATFORM_RANDOM", None);
    }

    // Step 7 — TLS / mbedtls. Manifest sets `mbedtls` to
    // `pkg-config` / `vendored` / `none`; bare-metal vendored path
    // pulls in the in-tree mbedTLS submodule's sources.
    if link.tls {
        match plat.mbedtls.as_deref() {
            Some("pkg-config") => {
                let pc_dir = out_dir.join("pkgconfig");
                generate_mbedtls_pc_files(&pc_dir);
                let existing = env::var("PKG_CONFIG_PATH").unwrap_or_default();
                let new_path = if existing.is_empty() {
                    pc_dir.display().to_string()
                } else {
                    format!("{}:{existing}", pc_dir.display())
                };
                // SAFETY: build scripts are single-threaded.
                unsafe { env::set_var("PKG_CONFIG_PATH", &new_path) };
                let lib = pkg_config::Config::new()
                    .cargo_metadata(true)
                    .probe("mbedtls")
                    .expect("mbedtls discovery via pkg-config failed");
                for include in &lib.include_paths {
                    build.include(include);
                }
            }
            Some("vendored") | None => {
                // Bare-metal default — pull vendor sources.
                let zpico_sys_dir = zenoh_pico_src.parent().unwrap();
                let mbedtls_dir = zpico_sys_dir.join("mbedtls");
                let mbedtls_include = mbedtls_dir.join("include");
                let mbedtls_library = mbedtls_dir.join("library");
                if !mbedtls_include.exists() {
                    panic!(
                        "mbedTLS source not provisioned at {:?}. Run: nros setup \
                         --source mbedtls (or git submodule update --init). — #0390",
                        mbedtls_include
                    );
                }
                build.include(&mbedtls_include);
                build.define("MBEDTLS_CONFIG_FILE", "\"mbedtls_config.h\"");
                let excluded = ["net_sockets.c", "timing.c", "threading.c", "psa_its_file.c"];
                if let Ok(entries) = std::fs::read_dir(&mbedtls_library) {
                    for entry in entries.flatten() {
                        let path = entry.path();
                        if path.extension().is_some_and(|ext| ext == "c") {
                            let fname = path.file_name().unwrap().to_str().unwrap();
                            if !excluded.contains(&fname) {
                                build.file(&path);
                            }
                        }
                    }
                }
            }
            Some("none") | Some(_) => {}
        }
    }

    // Step 8 — shim slot counts.
    shim.apply_to_cc(&mut build);

    // Step 9 — compile settings (opt_level / warnings / cflags).
    // Phase 204.9 — `opt_level` is numeric (`2`) or a string (`"s"`/`"z"`
    // for size); the size forms map to cc-rs `opt_level_str`.
    match &plat.compile.opt_level {
        Some(manifest::OptLevel::Num(level)) => {
            build.opt_level(*level);
        }
        Some(manifest::OptLevel::Str(level)) => {
            build.opt_level_str(level);
        }
        None => {}
    }
    if zpico_debug_requested() {
        build.debug(true);
    }
    if let Some(w) = plat.compile.warnings {
        build.warnings(w);
    } else {
        // Default warnings off across the manifest-driven path —
        // mirrors what every per-RTOS function used to set.
        build.warnings(false);
    }
    for flag in &plat.compile.cflags {
        build.flag(flag);
    }

    // Step 10 — PIC override (NuttX flat builds).
    if let Some(pic) = plat.pic {
        build.pic(pic);
    }

    // Step 11 — `link` field consumed by the policy layer earlier;
    // touch here so it doesn't go cold under the borrow checker.
    let _ = link;

    build.compile("zenohpico");

    // Step 12 — register additional rerun-if-env-changed hooks.
    for var in &plat.rerun_if_env_changed {
        println!("cargo:rerun-if-env-changed={var}");
    }
}

/// Generate pkg-config `.pc` files for mbedTLS.
///
/// Ubuntu's `libmbedtls-dev` doesn't ship `.pc` files, but zenoh-pico's
/// CMakeLists.txt uses `pkg_check_modules` to discover mbedTLS. We generate
/// minimal `.pc` files pointing to the system library paths so CMake can
/// find the installed libraries.
fn generate_mbedtls_pc_files(pc_dir: &Path) {
    std::fs::create_dir_all(pc_dir).unwrap();

    // issue 0399 (mbedtls half) — these .pc files are FABRICATED by us, so
    // `pkg_config::probe("mbedtls")` cannot fail: it reads back what we just
    // wrote and reports the library present whether or not it is installed.
    // The .pc hardcodes `includedir=/usr/include`, so on a host without the
    // dev package the build sails past discovery and dies ~40 lines later in a
    // vendored TU:
    //
    //     zenoh-pico/include/zenoh-pico/system/link/tls.h:30:10:
    //         fatal error: mbedtls/entropy.h: No such file or directory
    //
    // which reads as a broken vendored tree, not a missing system package.
    // Check the header we are about to promise before promising it.
    if !Path::new("/usr/include/mbedtls/entropy.h").exists() {
        panic!(
            "TLS is enabled but mbedTLS headers are missing \
             (/usr/include/mbedtls/entropy.h). Install the dev package:\n\
             \n    Debian/Ubuntu:  sudo apt-get install libmbedtls-dev\n\
             \n    Arch:           sudo pacman -S mbedtls\n\
             \n    Fedora:         sudo dnf install mbedtls-devel\n\
             \nor build without the `link-tls` feature."
        );
    }

    // Detect library directory (multi-arch on Debian/Ubuntu)
    let lib_dir = if Path::new("/usr/lib/x86_64-linux-gnu/libmbedtls.so").exists() {
        "/usr/lib/x86_64-linux-gnu"
    } else if Path::new("/usr/lib/aarch64-linux-gnu/libmbedtls.so").exists() {
        "/usr/lib/aarch64-linux-gnu"
    } else {
        "/usr/lib"
    };

    for (name, pc) in crate::mbedtls_pc_files(lib_dir) {
        std::fs::write(pc_dir.join(format!("{name}.pc")), pc).unwrap();
    }
}

/// Generate zenoh-pico version header for embedded builds.
fn generate_embedded_version_header(zenoh_pico_src: &Path, include_dir: &Path) {
    std::fs::create_dir_all(include_dir).unwrap();

    let version_file = zenoh_pico_src.join("version.txt");
    let version = std::fs::read_to_string(version_file)
        .unwrap_or_else(|_| "0.0.0".to_string())
        .trim()
        .to_string();

    let template = include_str!("../../zpico-sys/c/zenoh-pico-version.h.in");
    let header = crate::embedded_version_header(&version, template);

    std::fs::write(include_dir.join("zenoh-pico.h"), header).unwrap();
}

#[cfg(test)]
mod platform_watch_tests {
    use super::platform_manifests_to_watch;
    use std::path::PathBuf;

    /// A unique scratch dir, without a `tempfile` dev-dependency: adding one
    /// would move `Cargo.lock`, and a lockfile moves only when a dev means it.
    fn scratch(tag: &str) -> PathBuf {
        let d =
            std::env::temp_dir().join(format!("nros-platform-watch-{}-{tag}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn write_manifest(dir: &PathBuf) -> PathBuf {
        std::fs::create_dir_all(dir).unwrap();
        let m = dir.join("nros-platform.toml");
        std::fs::write(&m, "[platform.nuttx]\n").unwrap();
        m
    }

    /// The layout that was being missed: a platform PACKAGE directory is
    /// `nros-platform-<name>`, not `<name>`. Reconstructing
    /// `root/<name>/nros-platform.toml` produced a path that never existed, so
    /// the `is_file()` filter silently dropped every manifest under
    /// `packages/platform/` — and editing one changed nothing until an
    /// unrelated rebuild.
    #[test]
    fn a_platform_package_dir_is_watched_though_not_named_for_the_platform() {
        let root = scratch("pkg");

        // The shape that broke: directory name != platform name.
        let pkg = write_manifest(&root.join("nros-platform-nuttx"));
        // The shape that already worked, kept so the fix is not a swap.
        let cfg = write_manifest(&root.join("nuttx"));
        // No manifest: must NOT be watched — cargo treats a missing
        // `rerun-if-changed` input as permanently dirty (issue 0966).
        std::fs::create_dir_all(root.join("nros-platform-empty")).unwrap();

        let watched = platform_manifests_to_watch(std::slice::from_ref(&root));
        assert!(
            watched.contains(&pkg),
            "the `nros-platform-<name>` manifest must be watched — the case the \
             old reconstruction missed. got: {watched:?}"
        );
        assert!(watched.contains(&cfg), "got: {watched:?}");
        assert_eq!(watched.len(), 2, "only manifests that EXIST: {watched:?}");

        let _ = std::fs::remove_dir_all(&root);
    }

    /// A search PATH may name roots that are absent; normal, must not panic.
    #[test]
    fn a_missing_search_root_is_skipped() {
        let root = scratch("absent");
        let gone = root.join("not-here");
        assert!(platform_manifests_to_watch(std::slice::from_ref(&gone)).is_empty());
        let _ = std::fs::remove_dir_all(&root);
    }

    /// Documents the old bug directly: the reconstructed path does not exist
    /// for a package directory, which is why the watch was silent.
    #[test]
    fn the_old_reconstruction_would_have_found_nothing() {
        let root = scratch("recon");
        write_manifest(&root.join("nros-platform-nuttx"));

        let reconstructed = root.join("nuttx").join("nros-platform.toml");
        assert!(
            !reconstructed.is_file(),
            "if this exists, the regression this test documents is gone"
        );
        assert_eq!(
            platform_manifests_to_watch(std::slice::from_ref(&root)).len(),
            1
        );

        let _ = std::fs::remove_dir_all(&root);
    }
}
