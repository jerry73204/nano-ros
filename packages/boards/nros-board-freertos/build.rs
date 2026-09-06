//! Phase 152.1.B.4 — generic FreeRTOS + lwIP + nros-platform-freertos
//! build pipeline carved out of `nros-board-mps2-an385-freertos/build.rs`.
//!
//! Compiles four static archives that any FreeRTOS + lwIP overlay
//! links transitively:
//!
//! | Archive                  | Contents                                          |
//! |--------------------------|---------------------------------------------------|
//! | `libfreertos.a`          | kernel core + port + heap_4                       |
//! | `liblwip.a`              | core + IPv4 + API + netif + ethernet + sys_arch   |
//! | `libnros_platform_freertos.a` | C port providing the `nros_platform_*` ABI   |
//! | `libfreertos_glue.a`     | `c/freertos_hooks.c` + `c/network_glue.c` + `c/freertos_task_glue.c` + `c/freertos_run_tiers.c` |
//!
//! `c/freertos_c_entry.c` is deliberately NOT in that list: it is the C/C++
//! lane's boot path (`main`), and on the cargo lane `main` is the Rust entry.
//! The CMake board overlay compiles it via `FREERTOS_STARTUP_SOURCE`
//! (phase-337 W5.b).
//!
//! Required env vars (read from the user's environment; the
//! per-board overlay's `.cargo/config.toml [env]` block typically
//! sets them):
//!
//! | Var | Purpose | Required |
//! |---|---|---|
//! | `FREERTOS_DIR`        | FreeRTOS kernel source root | yes |
//! | `FREERTOS_PORT`       | Portable layer (e.g. `GCC/ARM_CM3`) | defaults to `GCC/ARM_CM3` |
//! | `LWIP_DIR`            | lwIP source root | yes |
//! | `FREERTOS_CONFIG_DIR` | Directory with `FreeRTOSConfig.h` + `lwipopts.h` | yes |
//! | `FREERTOS_CFLAGS`     | Space-separated cflags (`-mcpu=cortex-m3 -mthumb` etc.) | defaults to Cortex-M3 |
//!
//! The overlay's `build.rs` shrinks to: linker scripts into `OUT_DIR` +
//! board driver build (LAN9118 / STM ETH / NXP ENET / …) +
//! `c/board_<name>.c` (vector table + Reset + netif registration) + the
//! `NROS_APP_CONFIG` TU + libc/libgcc discovery +
//! `cargo:rustc-link-search`. The shared helpers it calls live in
//! `nros_board_common::freertos_build` (phase-337 W5.d).

use std::{env, path::PathBuf};

// phase-337 W5.d — cflags + include-dir setup are SHARED with the per-board
// overlays. Two copies had already drifted (the overlay hardcoded Cortex-M3
// while this crate resolved `[arch.*]`), so there is one spelling now.
use nros_board_common::freertos_build::{
    add_freertos_includes, add_lwip_includes, configure_cflags,
};
// issue 0491 — the ONE path-input helper, reached through the build-helper
// crate this script already deps (no new edge in any leaf lockfile).
use nros_board_common::nros_build_paths;

fn main() {
    // issue 0288 — the FREERTOS_DIR guard below does NOT cover host tooling:
    // an example's `.cargo/config.toml [env]` sets that var for a host probe
    // just as for a firmware build, so the guard passes and the ARM flags reach
    // the host compiler (`cc: error: unrecognized command-line option
    // '-mthumb'`). Only the target says what we are building FOR.
    if nros_board_common::host_probe::skip_cross_build("nros-board-freertos", &["thumb", "arm"]) {
        return;
    }

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // Skip the heavy lift unless the build is actually targeting a
    // FreeRTOS overlay. A `cargo check` of just this crate (no
    // downstream consumer + no env vars) panics today — gate the
    // env-var reads on the presence of `FREERTOS_DIR` so the bare
    // `cargo check -p nros-board-freertos` keeps working.
    if env::var("FREERTOS_DIR").is_err() {
        // Document via a build warning so an overlay author that
        // forgot to set the env var sees a clear hint instead of a
        // confusing missing-symbol error at link time.
        println!(
            "cargo:warning=nros-board-freertos: FREERTOS_DIR not set; \
             skipping kernel / lwIP / glue compile. Set it in your \
             overlay's `.cargo/config.toml [env]` block."
        );
        return;
    }

    // Canonical, so the `rerun-if-changed` lines these feed at the bottom read
    // the same from every consumer (issue 0491).
    let freertos_dir = env_path("FREERTOS_DIR");
    let freertos_port = env::var("FREERTOS_PORT").unwrap_or_else(|_| "GCC/ARM_CM3".to_string());
    let lwip_dir = env_path("LWIP_DIR");
    let freertos_config_dir = env_path("FREERTOS_CONFIG_DIR");
    let port_dir = freertos_dir.join("portable").join(&freertos_port);

    // --- Build FreeRTOS kernel ---
    let mut freertos = cc::Build::new();
    configure_cflags(&mut freertos);
    add_freertos_includes(
        &mut freertos,
        &freertos_dir,
        &port_dir,
        &freertos_config_dir,
    );
    // Phase 204.6 — right-size the FreeRTOS heap (heap_4 `ucHeap`, the dominant
    // bss; this is the only TU that sizes it). FreeRTOSConfig.h defaults to a
    // cyclone-safe 3 MiB. Two overrides, env wins:
    //   1. explicit `NROS_FREERTOS_HEAP_KB` build env (any value), else
    //   2. the `rmw-zenoh` feature (forwarded from the board) → 2 MiB. The
    //      FreeRTOS task stacks are allocated *from* this heap (heap_4), so it
    //      must hold the `nros_app` task stack (128 KiB since issue 1146 —
    //      MEASURED, worst in-tree peak 36 152 bytes; it was 384 KiB, and one
    //      task stack per TIER comes out of the same heap) PLUS lwIP
    //      (netconns/pbufs/socket semaphores) PLUS zenoh-pico's working set.
    //      512 KiB sufficed for the old *direct* talker but the Entry path
    //      MALLOC-FAILs at it (issue #46); 2 MiB boots cleanly through Executor
    //      + network on the qemu MPS2-AN385 (4 MiB SRAM, ample headroom). Still
    //      below the cyclone DDS-discovery default; cyclone/xrce don't enable
    //      this feature on the base crate, so they keep the 3 MiB default; tune
    //      via the env (`xPortGetMinimumEverFreeHeapSize()` high-water).
    // phase-400 W6 — the platform and board rungs sit UNDER the two overrides
    // this already had, and the env front-end still wins over all of them.
    //
    // The knob keeps its KiB spelling at the front end and the ladder stores
    // bytes, so the define is converted back here: FreeRTOSConfig.h reads KiB.
    let zenoh_default_kb =
        (env::var("CARGO_FEATURE_RMW_ZENOH").is_ok()).then(|| 2048_usize);
    let heap_kb = match nros_board_common::platform_config::BuildRungs::from_build_env() {
        Some(rungs) => {
            // No lane default means "leave FreeRTOSConfig.h's 3 MiB alone",
            // which is not a number this can invent — so only ask the ladder
            // when something below it has an opinion.
            zenoh_default_kb.map(|kb| rungs.memory_value("heap_bytes", kb * 1024) / 1024)
        }
        None => env::var("NROS_FREERTOS_HEAP_KB")
            .ok()
            .and_then(|v| v.trim().parse::<usize>().ok())
            .or(zenoh_default_kb),
    };
    if let Some(kb) = heap_kb {
        freertos.define("NROS_FREERTOS_HEAP_KB", kb.to_string().as_str());
    }
    println!("cargo:rerun-if-env-changed=NROS_FREERTOS_HEAP_KB");
    for src in &[
        "tasks.c",
        "queue.c",
        "list.c",
        "timers.c",
        "event_groups.c",
        "stream_buffer.c",
    ] {
        freertos.file(freertos_dir.join(src));
    }
    freertos.file(port_dir.join("port.c"));
    freertos.file(freertos_dir.join("portable/MemMang/heap_4.c"));
    // issue 0478 — cc-rs would hand arm-none-eabi-gcc the clang-only
    // `-mno-omit-leaf-frame-pointer`, which gcc REJECTS. These sites route
    // through neither shared helper, so the policy has to be named here.
    nros_cc_flags::gcc_safe_frame_pointer(&mut freertos);
    freertos.compile("freertos");

    // --- Build lwIP ---
    let mut lwip = cc::Build::new();
    configure_cflags(&mut lwip);
    add_freertos_includes(&mut lwip, &freertos_dir, &port_dir, &freertos_config_dir);
    add_lwip_includes(&mut lwip, &lwip_dir);
    for src in &[
        // Core
        "src/core/init.c",
        "src/core/def.c",
        "src/core/dns.c",
        "src/core/inet_chksum.c",
        "src/core/ip.c",
        "src/core/mem.c",
        "src/core/memp.c",
        "src/core/netif.c",
        "src/core/pbuf.c",
        "src/core/raw.c",
        "src/core/stats.c",
        "src/core/sys.c",
        "src/core/tcp.c",
        "src/core/tcp_in.c",
        "src/core/tcp_out.c",
        "src/core/timeouts.c",
        "src/core/udp.c",
        // IPv4 + IGMP for RTPS SPDP multicast
        "src/core/ipv4/etharp.c",
        "src/core/ipv4/icmp.c",
        "src/core/ipv4/ip4.c",
        "src/core/ipv4/ip4_addr.c",
        "src/core/ipv4/ip4_frag.c",
        "src/core/ipv4/igmp.c",
        // API (sockets)
        "src/api/api_lib.c",
        "src/api/api_msg.c",
        "src/api/err.c",
        "src/api/if_api.c",
        "src/api/netbuf.c",
        "src/api/netdb.c",
        "src/api/netifapi.c",
        "src/api/sockets.c",
        "src/api/tcpip.c",
        // Netif + FreeRTOS sys_arch
        "src/netif/ethernet.c",
        "contrib/ports/freertos/sys_arch.c",
    ] {
        lwip.file(lwip_dir.join(src));
    }
    // issue 0478 — cc-rs would hand arm-none-eabi-gcc the clang-only
    // `-mno-omit-leaf-frame-pointer`, which gcc REJECTS. These sites route
    // through neither shared helper, so the policy has to be named here.
    nros_cc_flags::gcc_safe_frame_pointer(&mut lwip);
    lwip.compile("lwip");

    // --- Build nros-platform-freertos C port ---
    // First-party sibling C source/headers. Resolved through the shared
    // path helper (issue 0491): env override first, otherwise the in-repo
    // default — the same answer `just/sdk-env.just` exports, so an unset
    // variable is no longer a panic. It WAS one, and that made
    // `logging-smoke-freertos-mps2` (the one FreeRTOS row with no `[env]`
    // block) unbuildable outside `just` — which `rust-fixture-stale.sh`, whose
    // stderr goes to /dev/null, read as "not stale" for a fixture whose build
    // could not even start.
    let nros_platform_freertos_dir = nros_build_paths::nros_platform_freertos_src();
    let nros_platform_cffi_include = nros_build_paths::nros_platform_cffi_include();
    let mut platform = cc::Build::new();
    configure_cflags(&mut platform);
    add_freertos_includes(
        &mut platform,
        &freertos_dir,
        &port_dir,
        &freertos_config_dir,
    );
    add_lwip_includes(&mut platform, &lwip_dir);
    platform.include(&nros_platform_cffi_include);
    platform.file(nros_platform_freertos_dir.join("platform.c"));
    platform.file(nros_platform_freertos_dir.join("net.c"));
    platform.file(nros_platform_freertos_dir.join("timer.c"));
    // issue 0478 — cc-rs would hand arm-none-eabi-gcc the clang-only
    // `-mno-omit-leaf-frame-pointer`, which gcc REJECTS. These sites route
    // through neither shared helper, so the policy has to be named here.
    nros_cc_flags::gcc_safe_frame_pointer(&mut platform);
    platform.compile("nros_platform_freertos");
    // issue 0491 — the CONTENT of the two first-party trees is what this build
    // depends on, and `watch_path` states it with the canonical spelling, so
    // every leaf that reaches this script agrees on the trigger.
    nros_build_paths::watch_path(&nros_platform_freertos_dir);
    nros_build_paths::watch_path(&nros_platform_cffi_include);

    // --- Generic glue (freertos_hooks + network_glue + freertos_run_tiers) ---
    // `c/freertos_hooks.c` provides the FreeRTOS task hooks +
    // semihosting helpers. `c/network_glue.c` provides the lwIP
    // init + FFI surface Rust calls; both invoke
    // `nros_board_*` weak hooks the overlay implements (152.1.B.2).
    // `c/freertos_run_tiers.c` (Phase 274.W3) implements
    // `nros_board_freertos_run_tiers` for embedded C/C++ multi-tier entries
    // (RFC-0015 Model 1): one FreeRTOS task per tier over one shared session.
    // nros-cpp C FFI symbols are forward-declared and resolved at link time.
    let mut glue = cc::Build::new();
    configure_cflags(&mut glue);
    add_freertos_includes(&mut glue, &freertos_dir, &port_dir, &freertos_config_dir);
    add_lwip_includes(&mut glue, &lwip_dir);
    glue.file(manifest_dir.join("c/freertos_hooks.c"));
    glue.file(manifest_dir.join("c/network_glue.c"));
    // phase-370 W1 — the kernel-only half of what `network_glue.c` used to be.
    // Split out so a board with no lwIP can compile it; this lane compiles both.
    glue.file(manifest_dir.join("c/freertos_task_glue.c"));
    glue.file(manifest_dir.join("c/freertos_run_tiers.c"));
    // issue 0478 — cc-rs would hand arm-none-eabi-gcc the clang-only
    // `-mno-omit-leaf-frame-pointer`, which gcc REJECTS. These sites route
    // through neither shared helper, so the policy has to be named here.
    nros_cc_flags::gcc_safe_frame_pointer(&mut glue);
    glue.compile("freertos_glue");

    // --- Link order (link-lib propagates transitively to overlays + final binary) ---
    println!("cargo:rustc-link-lib=static=nros_platform_freertos");
    println!("cargo:rustc-link-lib=static=freertos_glue");
    println!("cargo:rustc-link-lib=static=lwip");
    println!("cargo:rustc-link-lib=static=freertos");

    // --- Rerun triggers ---
    println!("cargo:rerun-if-changed=c/freertos_hooks.c");
    println!("cargo:rerun-if-changed=c/network_glue.c");
    println!("cargo:rerun-if-changed=c/freertos_run_tiers.c");
    println!("cargo:rerun-if-changed=build.rs");
    println!(
        "cargo:rerun-if-changed={}",
        freertos_config_dir.join("FreeRTOSConfig.h").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        freertos_config_dir.join("lwipopts.h").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        freertos_config_dir.join("arch/cc.h").display()
    );
    println!("cargo:rerun-if-changed={}", freertos_dir.display());
    println!("cargo:rerun-if-changed={}", lwip_dir.display());
    // issue 0491 — `FREERTOS_DIR` / `LWIP_DIR` / `FREERTOS_CONFIG_DIR` /
    // `NROS_PLATFORM_*` are PATHS and are deliberately NOT fingerprinted as
    // strings: cargo compares an env value textually, and one directory has a
    // different spelling per example leaf (`relative = true`), from `just`
    // (absolute) and from a bare build (unset). Their CONTENT is watched above
    // instead. `FREERTOS_PORT` (a port NAME) and `FREERTOS_CFLAGS` (a value)
    // stay — a change in either really is a different compile.
    println!("cargo:rerun-if-env-changed=FREERTOS_PORT");
    println!("cargo:rerun-if-env-changed=FREERTOS_CFLAGS");
}

/// A REQUIRED path variable, canonicalised (issue 0491) — never fingerprinted
/// as a string. The three callers below sit behind the `FREERTOS_DIR` guard,
/// so "unset" is already handled as "not a FreeRTOS build".
fn env_path(name: &str) -> PathBuf {
    let raw = PathBuf::from(env::var(name).unwrap_or_else(|_| {
        panic!(
            "{name} not set — overlays should set it via \
             `.cargo/config.toml [env]` or the user must export it"
        )
    }));
    nros_build_paths::canonical(&raw)
}
