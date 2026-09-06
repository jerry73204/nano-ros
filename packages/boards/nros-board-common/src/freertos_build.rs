//! Shared `build.rs` helpers for the FreeRTOS + lwIP board family.
//!
//! phase-337 W5.d — `configure_arm_cm3` / `add_freertos_includes` /
//! `add_lwip_includes` existed twice (`nros-board-freertos/build.rs` and
//! `nros-board-mps2-an385-freertos/build.rs`), and the copies had already
//! diverged: the family crate resolved cflags from the `[arch.*]` profiles
//! (phase-338 W4) while the overlay still hardcoded a Cortex-M3 fallback — so a
//! Cortex-M7 overlay would have compiled its own board glue with M3 flags while
//! the kernel beside it got M7 flags. One spelling lives here; both build
//! scripts call it (CLAUDE.md: add ONE shared helper, never a second spelling).
//!
//! [`emit_app_config_tu`] is the third de-duplication: the `NROS_APP_CONFIG` C
//! symbol was a 57-line hand-maintained C string in the overlay's `build.rs`
//! mirroring `nros_board_freertos::Config::default()` by eye. It now takes a
//! [`BaseConfig`] and a [`FreertosScheduling`] and writes the same TU from
//! them.

use std::{
    env,
    fs::File,
    io::Write,
    path::{Path, PathBuf},
};

use crate::{
    arch_flags,
    base_config::BaseConfig,
    freertos_config::{FreertosScheduling, app_stack_bytes},
};

/// The platform whose `[arch.*]` profiles supply this family's cflags —
/// `config/freertos/nros-platform.toml`.
// phase-349 W1 — the platform is `freertos`; the stack is a fact declared
// elsewhere. `freertos-lwip` survives as an alias in the descriptor's `names`.
const PLATFORM: &str = "freertos";

/// Shared cflag setup for every FreeRTOS + lwIP translation unit.
///
/// `-ffunction-sections` / `-fdata-sections` / `-O2` / warnings-off stay
/// built-in defaults — every FreeRTOS+lwIP consumer wants them.
///
/// Resolution order for the target flags, so the RFC-0049 ladder still runs
/// board-over-platform:
///   1. `FREERTOS_CFLAGS` — the board's explicit override (rung 1).
///   2. the matching `[arch.*]` profile for this `TARGET`.
///   3. loud failure naming the profiles that exist and what they admit.
///
/// A non-`thumb*` target (host `cargo check`, the source-metadata probe) skips
/// all of it — there is no embedded compile of substance there.
///
/// # Panics
/// When a `thumb*` target matches no `[arch.*]` profile and no
/// `FREERTOS_CFLAGS` was given. Silently compiling for the wrong CPU or FPU ABI
/// is worse than failing here (phase-195 audit (b)).
pub fn configure_cflags(build: &mut cc::Build) {
    build
        .opt_level(2)
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .warnings(false);
    // issue 0383 — implicit-function-declaration / int-conversion as ERRORS.
    // Safe next to `warnings(false)`: that only makes cc-rs OMIT `-Wall`/
    // `-Wextra`, it passes no `-w`, and gcc enables both diagnostics by
    // default — so the gate is live on the pinned arm-none-eabi-gcc 13.2.
    nros_cc_flags::strict_decls(build);

    for flag in resolve_cflags().split_whitespace() {
        build.flag(flag);
    }
}

fn resolve_cflags() -> String {
    if let Ok(v) = env::var("FREERTOS_CFLAGS") {
        return v;
    }
    let target = env::var("TARGET").unwrap_or_default();
    // phase-372 W1 — `arm*` cross targets (Cortex-R52: `armv8r-none-eabihf`)
    // resolve through the [arch.*] profiles exactly like `thumb*` ones. The
    // old guard returned the M3 legacy default for ANYTHING non-thumb, which
    // would silently compile R-profile boards with `-mcpu=cortex-m3 -mthumb`
    // — the wrong-CPU outcome the panic below exists to prevent. Host builds
    // (x86_64…) still take the legacy default; they never reach a real
    // embedded compile (skip_cross_build guards the board build scripts).
    if !target.starts_with("thumb") && !target.starts_with("arm") {
        return "-mcpu=cortex-m3 -mthumb".to_string();
    }
    let config_root = arch_flags::config_root().unwrap_or_else(|| {
        panic!(
            "nros-board-freertos: TARGET=`{target}` needs arch cflags but the nano-ros \
             config/ tree was not found walking up from CARGO_MANIFEST_DIR. Out-of-tree \
             consumer? Set FREERTOS_CFLAGS explicitly."
        )
    });
    match arch_flags::cflags_for_target(&config_root, PLATFORM, &target) {
        Ok(Some(flags)) => flags.join(" "),
        Ok(None) => panic!(
            "nros-board-freertos: no [arch.*] profile of platform `{PLATFORM}` admits \
             TARGET=`{target}`.\n  declared: {}\n  Either add an [arch.*] block to \
             config/{PLATFORM}/nros-platform.toml, or set FREERTOS_CFLAGS in the board's \
             .cargo/config.toml [env] — e.g. `-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 \
             -mfloat-abi=hard` for a Cortex-M4F.",
            arch_flags::describe_profiles(&config_root, PLATFORM)
        ),
        Err(e) => panic!("nros-board-freertos: reading arch profiles: {e}"),
    }
}

/// FreeRTOS kernel + port + `FreeRTOSConfig.h` include dirs.
pub fn add_freertos_includes(
    build: &mut cc::Build,
    freertos_dir: &Path,
    port_dir: &Path,
    config_dir: &Path,
) {
    build
        .include(config_dir)
        .include(freertos_dir.join("include"))
        .include(port_dir);
}

/// lwIP core + FreeRTOS contrib-port include dirs.
pub fn add_lwip_includes(build: &mut cc::Build, lwip_dir: &Path) {
    build
        .include(lwip_dir.join("src/include"))
        .include(lwip_dir.join("contrib/ports/freertos/include"));
}

/// The app-task stack size for this build, honouring
/// `NROS_FREERTOS_APP_STACK_KB`. Emits the `rerun-if-env-changed` line.
pub fn app_stack_bytes_from_build_env() -> u32 {
    println!("cargo:rerun-if-env-changed=NROS_FREERTOS_APP_STACK_KB");
    let builtin = app_stack_bytes(None);
    // phase-400 W6 — the platform and board rungs, beneath the env front-end
    // that still wins. `None` when no lane named a platform, which is a bare
    // `cargo build`: then this is exactly the env-or-default it always was.
    match crate::platform_config::BuildRungs::from_build_env() {
        Some(rungs) => rungs.memory_value("app_stack_bytes", builtin as usize) as u32,
        None => app_stack_bytes(env::var("NROS_FREERTOS_APP_STACK_KB").ok().as_deref()),
    }
}

/// Write the board's `NROS_APP_CONFIG` definition into `out_dir` and return the
/// path, for `cc::Build::file`.
///
/// The symbol is what the C/C++ application entry reads for network bring-up
/// and task sizing (`<nros/app_config.h>` declares the type). Before phase-337
/// W5.d this was a 57-line C string literal in the MPS2 overlay's `build.rs`,
/// maintained by eye against `nros_board_freertos::Config::default()` — and it
/// had drifted by 128 KiB on `app_stack_bytes`.
pub fn emit_app_config_tu(
    out_dir: &Path,
    base: &BaseConfig,
    sched: &FreertosScheduling,
) -> PathBuf {
    let out_path = out_dir.join("nros_app_config_def.c");
    let ip = base.ip;
    let mac = base.mac;
    let gw = base.gateway;
    let nm = base.netmask;
    let body = format!(
        r#"/* GENERATED by nros_board_common::freertos_build::emit_app_config_tu —
 * do not edit. The values come from the board crate's `BaseConfig` +
 * `FreertosScheduling`, so this file cannot drift from the Rust defaults the
 * way the hand-written mirror it replaces did (phase-337 W5.d).
 *
 * `<nros/app_config.h>` is the canonical-path wrapper for the shipped
 * `nros_app_config_t` type, so there is no inlined-typedef sync obligation.
 */

#include <stdint.h>
#include <nros/app_config.h>

const nros_app_config_t NROS_APP_CONFIG = {{
    .zenoh = {{
        .locator   = "{locator}",
        .domain_id = {domain_id},
    }},
    .network = {{
        .ip      = {{ {ip0}, {ip1}, {ip2}, {ip3} }},
        .mac     = {{ 0x{mac0:02x}, 0x{mac1:02x}, 0x{mac2:02x}, 0x{mac3:02x}, 0x{mac4:02x}, 0x{mac5:02x} }},
        .gateway = {{ {gw0}, {gw1}, {gw2}, {gw3} }},
        .netmask = {{ {nm0}, {nm1}, {nm2}, {nm3} }},
        .prefix  = {prefix},
    }},
    .scheduling = {{
        .app_priority            = {app_priority},
        .zenoh_read_priority     = {zenoh_read_priority},
        .zenoh_lease_priority    = {zenoh_lease_priority},
        .poll_priority           = {poll_priority},
        .app_stack_bytes         = {app_stack_bytes}u,
        .zenoh_read_stack_bytes  = {zenoh_read_stack_bytes}u,
        .zenoh_lease_stack_bytes = {zenoh_lease_stack_bytes}u,
        .poll_interval_ms        = {poll_interval_ms}u,
    }},
}};
"#,
        locator = base.zenoh_locator,
        domain_id = base.domain_id,
        ip0 = ip[0],
        ip1 = ip[1],
        ip2 = ip[2],
        ip3 = ip[3],
        mac0 = mac[0],
        mac1 = mac[1],
        mac2 = mac[2],
        mac3 = mac[3],
        mac4 = mac[4],
        mac5 = mac[5],
        gw0 = gw[0],
        gw1 = gw[1],
        gw2 = gw[2],
        gw3 = gw[3],
        nm0 = nm[0],
        nm1 = nm[1],
        nm2 = nm[2],
        nm3 = nm[3],
        prefix = base.prefix(),
        // Issue 0623 — the struct already holds RAW FreeRTOS priorities, so
        // this emits them verbatim.
        //
        // It converted here for one commit, while the struct was still
        // normalized; the conversion has since moved to the only place that can
        // know which scale a number is on — the parser that reads it from a
        // `[node.rt]` (normalized, legacy) or `[node.rt.freertos]` (raw)
        // section. Converting at the emitter meant every OTHER consumer had to
        // convert too, which is how the C entry's saturating `clamp_prio` came
        // to disagree with the Rust entry's proportional map (16 -> 7 vs
        // 16 -> 4, flattening app/transport/poll into one priority).
        app_priority = sched.app_priority,
        zenoh_read_priority = sched.zenoh_read_priority,
        zenoh_lease_priority = sched.zenoh_lease_priority,
        poll_priority = sched.poll_priority,
        app_stack_bytes = sched.app_stack_bytes,
        zenoh_read_stack_bytes = sched.zenoh_read_stack_bytes,
        zenoh_lease_stack_bytes = sched.zenoh_lease_stack_bytes,
        poll_interval_ms = sched.poll_interval_ms,
    );
    File::create(&out_path)
        .expect("failed to create nros_app_config_def.c")
        .write_all(body.as_bytes())
        .expect("failed to write nros_app_config_def.c");
    out_path
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_emitted_tu_carries_the_config_it_was_given() {
        let dir = tempfile::tempdir().unwrap();
        let path = emit_app_config_tu(
            dir.path(),
            &BaseConfig::default(),
            &FreertosScheduling::default(),
        );
        let tu = std::fs::read_to_string(path).unwrap();
        assert!(tu.contains(r#".locator   = "tcp/192.0.3.1:7447""#), "{tu}");
        assert!(tu.contains(".ip      = { 192, 0, 3, 10 }"), "{tu}");
        assert!(
            tu.contains(".mac     = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 }"),
            "{tu}"
        );
        assert!(tu.contains(".prefix  = 24,"), "{tu}");
        // The number the hand-written mirror got wrong.
        assert!(tu.contains(".app_stack_bytes         = 131072u,"), "{tu}");
    }
}
