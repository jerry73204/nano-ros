//! phase-338 W4 — resolve `[arch.*]` compiler flags for a target triple.
//!
//! The `[arch.*]` profiles in `config/<platform>/nros-platform.toml` already
//! carry the `cflags` a target needs (RFC-0049). Before this module every
//! consumer either re-implemented the match or hardcoded one arch's flags and
//! **panicked** on the rest — which is how FreeRTOS+lwIP stayed unbuildable on
//! Cortex-M4F/M7 after `[arch.cortex-m7]` had already been added to the
//! platform config.
//!
//! One spelling of the predicate lives here, beside [`ArchEntry`] itself.
//! `nros_zpico_build::arch_matches` delegates to [`arch_matches`] rather than
//! carrying a second copy (CLAUDE.md: add ONE shared helper, never a second
//! spelling).

use std::path::{Path, PathBuf};

use crate::{manifest::ArchEntry, platform_config::PlatformsTree};

/// Returns `true` when the `[arch.*]` predicates admit this target triple.
///
/// Both predicates are substring tests on the triple: `target_match` must be
/// present, `target_exclude` must not. That is what disambiguates Cortex-M3
/// (`thumbv7m`) from Cortex-M4/M7 (`thumbv7em`), which share a prefix.
pub fn arch_matches(arch: &ArchEntry, target: &str) -> bool {
    if let Some(needle) = arch.target_match.as_deref()
        && !target.contains(needle)
    {
        return false;
    }
    if let Some(needle) = arch.target_exclude.as_deref()
        && target.contains(needle)
    {
        return false;
    }
    true
}

/// Walk up from `CARGO_MANIFEST_DIR` to the checkout root (the directory
/// holding `nros-sdk-index.toml`), then to its `config/` tree.
///
/// Returns `None` for an out-of-tree consumer, whose caller must fall back to
/// an explicit env var rather than guessing.
pub fn config_root() -> Option<PathBuf> {
    let start = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let mut dir = PathBuf::from(start);
    loop {
        if dir.join("nros-sdk-index.toml").is_file() {
            // phase-400 W1 — descriptors live beside their crates now, with
            // `config/` kept for the platforms that have no package. Prefer the
            // first root that actually holds one rather than assuming `config/`,
            // which after the move can exist and be empty of descriptors.
            for root in ["packages/platform", "config"] {
                let cand = dir.join(root);
                if cand.is_dir()
                    && std::fs::read_dir(&cand).is_ok_and(|mut e| {
                        e.any(|x| x.is_ok_and(|x| x.path().join("nros-platform.toml").is_file()))
                    })
                {
                    return Some(cand);
                }
            }
            return None;
        }
        if !dir.pop() {
            return None;
        }
    }
}

/// The `cflags` of the first `[arch.*]` profile of `platform` that admits
/// `target`, in the platform manifest's declared `arch = [..]` order.
///
/// `Ok(None)` means the platform declares profiles but none matches — a real
/// answer ("this platform does not claim to support this arch"), which the
/// caller should report rather than paper over with a default.
pub fn cflags_for_target(
    config_root: &Path,
    platform: &str,
    target: &str,
) -> Result<Option<Vec<String>>, String> {
    let tree = PlatformsTree::load(config_root).map_err(|e| e.to_string())?;
    let table = tree.arch_table().clone();
    for name in declared_arch_names(&tree, platform) {
        let Some(entry) = table.get(&name) else {
            // A name in `arch = [..]` with no `[arch.<name>]` block is a
            // manifest bug. Report it rather than silently trying the next
            // profile, which would pick the wrong flags.
            return Err(format!(
                "platform `{platform}` declares arch profile `{name}` but \
                 config/*/nros-platform.toml defines no [arch.{name}] block"
            ));
        };
        if arch_matches(entry, target) {
            return Ok(Some(entry.cflags.clone()));
        }
    }
    Ok(None)
}

/// The arch profile names `platform` declares, in `arch = [..]` order.
///
/// Order matters: the predicates are substring tests, so `cortex-m3`
/// (`thumbv7m`) must be offered before a profile that could also admit the
/// triple. First match wins, as in the zpico include-path resolver.
pub fn declared_arch_names(tree: &PlatformsTree, platform: &str) -> Vec<String> {
    // phase-349 W1 — the manifest is keyed by DIRECTORY, so an alias
    // (`freertos-lwip` after the directory became `freertos`) has to be
    // resolved first. This path does NOT go through `PlatformsTree::chain()`,
    // which is where the other lookups get alias handling for free — the
    // arch table is merged across all files and addressed separately. Caught by
    // `freertos_lwip_resolves_both_declared_arches`, which kept using the alias
    // for exactly this reason.
    let dir = tree.resolve_alias(platform);
    tree.as_platform_manifest()
        .platform
        .get(dir)
        .map(|entry| entry.arch.clone())
        .unwrap_or_default()
}

/// Diagnostic form: the profile names and what each would admit, for an error
/// message that tells the reader what to do rather than only what failed.
pub fn describe_profiles(config_root: &Path, platform: &str) -> String {
    let Ok(tree) = PlatformsTree::load(config_root) else {
        return "<platform config unreadable>".to_string();
    };
    let table = tree.arch_table().clone();
    let names = declared_arch_names(&tree, platform);
    if names.is_empty() {
        return format!("platform `{platform}` declares no [arch.*] profiles");
    }
    names
        .iter()
        .map(|n| match table.get(n) {
            Some(e) => format!(
                "{n} (match={:?}, exclude={:?})",
                e.target_match.as_deref().unwrap_or("*"),
                e.target_exclude.as_deref().unwrap_or("-")
            ),
            None => format!("{n} (UNDEFINED)"),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipped `config/freertos` profiles must resolve for both arches the
    /// platform declares. phase-338 W4: `[arch.cortex-m7]` was added and then
    /// ignored by the consumer, so FreeRTOS+lwIP stayed unbuildable on
    /// Cortex-M4F/M7 — this asserts the declaration is actually reachable.
    ///
    /// phase-349 W1 — deliberately still addressed as `freertos-lwip`, the
    /// ALIAS, after the directory became `freertos`. That keeps the alias path
    /// covered, and it immediately earned it: `declared_arch_names` does not go
    /// through `chain()` and was alias-blind until this test failed.
    #[test]
    fn freertos_lwip_resolves_both_declared_arches() {
        let root = config_root().expect("in-tree checkout");

        let m3 = cflags_for_target(&root, "freertos-lwip", "thumbv7m-none-eabi")
            .expect("readable")
            .expect("thumbv7m must select an arch profile");
        assert!(
            m3.iter().any(|f| f == "-mcpu=cortex-m3"),
            "thumbv7m selected {m3:?}, expected the cortex-m3 profile"
        );

        let m7 = cflags_for_target(&root, "freertos-lwip", "thumbv7em-none-eabihf")
            .expect("readable")
            .expect("thumbv7em-none-eabihf must select an arch profile — the M7 blocker");
        assert!(
            m7.iter().any(|f| f == "-mcpu=cortex-m7"),
            "thumbv7em-none-eabihf selected {m7:?}, expected the cortex-m7 profile"
        );
        assert!(
            m7.iter().any(|f| f == "-mfloat-abi=hard"),
            "the hard-float triple must select hard-float flags, got {m7:?}"
        );
    }

    /// An arch nobody declared returns `None` — a real answer the caller turns
    /// into a message naming what IS declared, never a silent wrong default.
    #[test]
    fn undeclared_arch_is_none_not_a_default() {
        let root = config_root().expect("in-tree checkout");
        let got =
            cflags_for_target(&root, "freertos-lwip", "thumbv6m-none-eabi").expect("readable");
        assert!(
            got.is_none(),
            "thumbv6m (Cortex-M0) is not declared by freertos-lwip; got {got:?}"
        );
    }

    /// phase-418 418.3 — the Orin SPE profile resolves for the SOFT-float
    /// ARMv7-R triple, and carries the vendor FSP's `softfp` flags.
    ///
    /// The vendor BSP (`spe-freertos-bsp/rt-aux-cpu-demo-fsp/Makefile`, L4T
    /// 36.4.4) compiles AND links `-mcpu=cortex-r5 -mthumb-interwork
    /// -mfloat-abi=softfp -mfpu=vfpv3-d16`. Our objects join that image, so
    /// the ABI is not ours to choose.
    #[test]
    fn freertos_resolves_cortex_r5_with_the_vendor_softfp_abi() {
        let root = config_root().expect("in-tree checkout");
        let r5 = cflags_for_target(&root, "freertos", "armv7r-none-eabi")
            .expect("readable")
            .expect("armv7r-none-eabi must select an arch profile — 418.3");
        assert!(
            r5.iter().any(|f| f == "-mcpu=cortex-r5"),
            "armv7r-none-eabi selected {r5:?}, expected the cortex-r5 profile"
        );
        assert!(
            r5.iter().any(|f| f == "-mfloat-abi=softfp"),
            "the SPE FSP is softfp; got {r5:?}"
        );
        assert!(
            !r5.iter().any(|f| f == "-mfloat-abi=hard"),
            "hard float against a soft-float-ABI FSP passes floats in the \
             wrong registers; got {r5:?}"
        );
    }

    /// `armv7r-none-eabi` is a SUBSTRING of `armv7r-none-eabihf`, so without
    /// `target_exclude = "eabihf"` the soft-float profile would silently claim
    /// the hard-float triple. Selecting nothing is the correct answer: the
    /// FreeRTOS platform does not describe a hard-float ARMv7-R C half.
    #[test]
    fn cortex_r5_does_not_claim_the_hard_float_triple() {
        let root = config_root().expect("in-tree checkout");
        let got = cflags_for_target(&root, "freertos", "armv7r-none-eabihf").expect("readable");
        assert!(
            got.is_none(),
            "armv7r-none-eabihf must select no profile, not the softfp one; got {got:?}"
        );
    }

    /// ARMv7-R and ARMv8-R must not claim each other. `armv8r` is the
    /// discriminator and it appears in neither armv7r triple.
    #[test]
    fn cortex_r5_and_r52_do_not_claim_each_others_triples() {
        let root = config_root().expect("in-tree checkout");
        let r52 = cflags_for_target(&root, "freertos", "armv8r-none-eabihf")
            .expect("readable")
            .expect("armv8r-none-eabihf must still select cortex-r52");
        assert!(
            r52.iter().any(|f| f == "-mcpu=cortex-r52"),
            "armv8r-none-eabihf selected {r52:?}, expected the cortex-r52 profile"
        );
    }

    /// `target_exclude` is what keeps M3 from claiming the M4/M7 triple.
    #[test]
    fn exclude_predicate_separates_m3_from_m7() {
        let m3 = ArchEntry {
            target_match: Some("thumbv7m".into()),
            target_exclude: Some("thumbv7em".into()),
            ..Default::default()
        };
        assert!(arch_matches(&m3, "thumbv7m-none-eabi"));
        assert!(
            !arch_matches(&m3, "thumbv7em-none-eabihf"),
            "cortex-m3 must not claim the M4F/M7 triple — that is the wrong-FPU-ABI bug"
        );
    }
}
