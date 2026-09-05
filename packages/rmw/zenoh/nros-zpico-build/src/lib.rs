//! Pure and testable build-script helpers for `zpico-sys`.

use std::{
    collections::HashSet,
    fmt::Write as _,
    path::{Path, PathBuf},
    process::Command,
};

use nros_board_common::{manifest, policy::LinkFeatures};

pub mod runner;

/// Shim slot count configuration.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ShimConfig {
    pub max_publishers: usize,
    pub max_subscribers: usize,
    pub max_queryables: usize,
    /// phase-392 W5.e — whether [`ShimConfig::max_queryables`] was sized from a
    /// DECLARATION (the resolved SystemModel, via `nros ws entity-facts`) or
    /// from the backend's own budget.
    ///
    /// The runtime needs to know which, because the two produce the same
    /// failure for different reasons. Sized from a budget, an exhausted table
    /// means "raise the knob". Sized from a declaration, it means the image
    /// created a service server its model does not declare — and saying "raise
    /// the knob" there sends the reader to fix the symptom. Being authoritative
    /// costs this, and phase-392 W5.b2 says it is paid in the same wave.
    pub queryable_table_declared: bool,
    pub max_liveliness: usize,
    pub max_pending_gets: usize,
    /// phase-328 (issue 0348) — size of the C shim's session pool
    /// (`ZPICO_MAX_SESSIONS`, default 1). A single-session target keeps the
    /// default so its static footprint is one session's tables plus one
    /// in-use flag; a multi-domain build raises it.
    pub max_sessions: usize,
    pub get_reply_buf_size: usize,
    pub get_poll_interval_ms: usize,
    /// phase-279 (#145) — opt-in tx batching (`ZPICO_TX_BATCH=1`): puts queue in
    /// the transport write buffer and ship as ONE socket send per
    /// `zpico_spin_once` flush. Default OFF (adds up to one spin period of tx
    /// latency; gets/replies/express publishers bypass). Biggest win on Zephyr,
    /// where zsock's per-fd lock caps tx at ~1 send per socket-recv window.
    pub tx_batch: bool,
    /// phase-282 (#145) — flush cadence, ms (`ZPICO_TX_BATCH_FLUSH_MS`,
    /// default 50): period of the dedicated tx-flush thread and rate limit of
    /// the spin-driven fallback flush. Bounds the extra publish latency the
    /// batch adds. Only meaningful with `tx_batch`.
    pub tx_batch_flush_ms: usize,
    /// issue 0626 / 0460 — priority of zenoh-pico's read task
    /// (`ZPICO_READ_TASK_PRIORITY`, default 16, from
    /// `CONFIG_NROS_ZENOH_READ_PRIORITY`).
    ///
    /// Here rather than only in `nros_rmw_zenoh.cmake` because that file is the
    /// C lane, and a Zephyr RUST image inherits none of cmake's
    /// `set(ENV{...})` exports — issue 0460's class, where the two lanes
    /// disagreed on `MAX_QUERYABLES` and the ABI split silently. The value is
    /// NORMALISED 0–31 and mapped down onto the platform's own range, not a
    /// raw RTOS priority; CLAUDE.md records the collision that follows from
    /// confusing the two units (issue 0623).
    pub read_task_priority: usize,
    /// issue 0626 / 0460 — priority of zenoh-pico's lease task
    /// (`ZPICO_LEASE_TASK_PRIORITY`, default 16, from
    /// `CONFIG_NROS_ZENOH_LEASE_PRIORITY`). See [`ShimConfig::read_task_priority`].
    pub lease_task_priority: usize,
}

/// Whether the generated zenoh-pico config compiles in the multicast transport
/// and scouting that zenoh's PEER mode is built on.
///
/// Issue 0682 — this is `false`, and that is a size decision, not an oversight:
/// multicast transport, scouting and multicast declarations are three more code
/// paths in a library whose whole point here is fitting on an MCU, and every
/// nano-ros deployment reaches its peers through a router or an agent. It is
/// named rather than written as a bare `0` because the value has to be
/// legible from BOTH sides — the C `#define` the library is compiled with, and
/// the Rust const the session layer checks before it tries to open a peer
/// session. When it was only the `0`, `NROS_SESSION_MODE=peer` failed as an
/// opaque `ConnectionFailed` and the book went on promising peer-to-peer.
///
/// Turning it on means flipping this, rebuilding the shim, and accepting the
/// footprint; nothing else here special-cases it.
pub const MULTICAST_TRANSPORT: bool = false;

/// The env var that overrides [`MULTICAST_TRANSPORT`] for one build.
///
/// The default stays `false`, so no shipped target moves. What this adds is a
/// way to build the OTHER configuration without editing source.
/// `nano2nano::test_peer_mode_communication` could only ever SKIP, because the
/// one build in the tree refuses peer mode by construction, and a capability
/// nothing can exercise is a capability nobody is testing — the argument issue
/// 0393 made for `ZPICO_MAX_SESSIONS`.
///
/// This is the build-input half. It is NOT yet enough to run that e2e test:
/// `nano2nano` spawns PREBUILT fixture binaries, which `build-test-fixtures`
/// produces without this flag, so exercising peer mode end to end needs a
/// fixture VARIANT rather than an env on the test crate. Issue 0711.
pub const MULTICAST_TRANSPORT_ENV: &str = "ZPICO_MULTICAST_TRANSPORT";

/// Whether THIS build compiles in multicast transport + scouting.
///
/// Reads [`MULTICAST_TRANSPORT_ENV`], falling back to [`MULTICAST_TRANSPORT`].
/// Every emitter goes through here — the three C `#define`s and the Rust
/// `ZPICO_PEER_MODE_SUPPORTED` const — so the C library and the Rust session
/// layer cannot disagree about what was compiled. That property is the whole
/// point of issue 0682 and it survives becoming configurable only because
/// there is still exactly ONE reader of the value.
pub fn multicast_transport_enabled() -> bool {
    match std::env::var(MULTICAST_TRANSPORT_ENV) {
        Ok(v) => !matches!(v.trim(), "" | "0" | "false" | "FALSE" | "off" | "OFF"),
        Err(_) => MULTICAST_TRANSPORT,
    }
}

/// The `0`/`1` spelling of [`multicast_transport_enabled`] for a C `#define`.
pub fn multicast_transport_flag() -> u8 {
    if multicast_transport_enabled() { 1 } else { 0 }
}

impl ShimConfig {
    /// Generate `$OUT_DIR/shim_constants.rs` contents with Rust const declarations.
    pub fn rust_consts(&self) -> String {
        format!(
            "/// Maximum number of concurrent publishers (set via ZPICO_MAX_PUBLISHERS, default 8).\n\
             pub const ZPICO_MAX_PUBLISHERS: usize = {};\n\
             /// Maximum number of concurrent subscribers (set via ZPICO_MAX_SUBSCRIBERS, default 8).\n\
             pub const ZPICO_MAX_SUBSCRIBERS: usize = {};\n\
             /// Maximum number of concurrent queryables (set via ZPICO_MAX_QUERYABLES, default 8).\n\
             pub const ZPICO_MAX_QUERYABLES: usize = {};\n\
             /// Maximum number of concurrent liveliness tokens (set via ZPICO_MAX_LIVELINESS, default 16).\n\
             pub const ZPICO_MAX_LIVELINESS: usize = {};\n\
             /// Maximum number of concurrent pending get operations (set via ZPICO_MAX_PENDING_GETS, default 4).\n\
             pub const ZPICO_MAX_PENDING_GETS: usize = {};\n\
             /// Size of the session pool (set via ZPICO_MAX_SESSIONS, default 1). phase-328 / issue 0348.\n\
             pub const ZPICO_MAX_SESSIONS: usize = {};\n\
             /// phase-392 W5.e — whether the queryable table was sized from the entry's\n\
             /// DECLARATION rather than from the backend's own budget. Decides which\n\
             /// exhaustion message the shim gives: a budget is raised, a declaration is\n\
             /// corrected.\n\
             pub const ZPICO_QUERYABLE_TABLE_DECLARED: bool = {};\n\
             /// Whether this shim was compiled with the multicast transport + scouting\n\
             /// that zenoh PEER mode needs (issue 0682). Emitted from the SAME constant\n\
             /// that writes the C `#define`, so the two cannot drift.\n\
             pub const ZPICO_PEER_MODE_SUPPORTED: bool = {};\n",
            self.max_publishers,
            self.max_subscribers,
            self.max_queryables,
            self.max_liveliness,
            self.max_pending_gets,
            self.max_sessions,
            self.queryable_table_declared,
            multicast_transport_enabled(),
        )
    }

    /// The `-D` flags the C shim must be compiled with, as data.
    ///
    /// Split out from [`ShimConfig::apply_to_cc`] so it is testable: `cc::Build`
    /// exposes no way to read back what was defined on it, so a test written
    /// against the builder can only assert that the call did not panic. Issue
    /// 0626's knobs reached the C lane and not this one for exactly as long as
    /// nobody could write that assertion.
    pub fn defines(&self) -> Vec<(&'static str, String)> {
        let mut out = vec![
            ("ZPICO_MAX_PUBLISHERS", self.max_publishers.to_string()),
            ("ZPICO_MAX_SUBSCRIBERS", self.max_subscribers.to_string()),
            ("ZPICO_MAX_QUERYABLES", self.max_queryables.to_string()),
            ("ZPICO_MAX_LIVELINESS", self.max_liveliness.to_string()),
            ("ZPICO_MAX_PENDING_GETS", self.max_pending_gets.to_string()),
            ("ZPICO_MAX_SESSIONS", self.max_sessions.to_string()),
            (
                "ZPICO_GET_REPLY_BUF_SIZE",
                self.get_reply_buf_size.to_string(),
            ),
            (
                "ZPICO_GET_POLL_INTERVAL_MS",
                self.get_poll_interval_ms.to_string(),
            ),
        ];
        if self.tx_batch {
            out.push(("ZPICO_TX_BATCH", "1".to_string()));
            out.push((
                "ZPICO_TX_BATCH_FLUSH_MS",
                self.tx_batch_flush_ms.to_string(),
            ));
        }
        // Unconditional, matching the C lane in `nros_rmw_zenoh.cmake`: the
        // shim `#define`s its own 16/16 fallback, so omitting these is not "no
        // opinion" — it is the C default silently winning over Kconfig, on the
        // Rust lane only. Issue 0460's shape (issue 0626's knobs).
        out.push((
            "ZPICO_READ_TASK_PRIORITY",
            self.read_task_priority.to_string(),
        ));
        out.push((
            "ZPICO_LEASE_TASK_PRIORITY",
            self.lease_task_priority.to_string(),
        ));
        out
    }

    /// Add `-D` flags to a `cc::Build` so the C shim picks up the same values.
    pub fn apply_to_cc(&self, build: &mut cc::Build) {
        for (name, value) in self.defines() {
            build.define(name, value.as_str());
        }
    }
}

/// Buffer size configuration for zenoh-pico.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ZenohBufferConfig {
    pub frag_max_size: usize,
    pub batch_unicast_size: usize,
    pub batch_multicast_size: usize,
}

/// Generate a zenoh-pico config header body from already-resolved build inputs.
pub fn config_header(
    link: &LinkFeatures,
    buf: &ZenohBufferConfig,
    target: &str,
    unstable_api: bool,
    tx_batch: bool,
    tx_split_lock: bool,
) -> String {
    let mut header = String::new();
    writeln!(header, "/**").unwrap();
    writeln!(header, " * zenoh_generic_config.h - Generated by build.rs").unwrap();
    writeln!(header, " *").unwrap();
    writeln!(
        header,
        " * Z_FEATURE_LINK_* values are derived from Cargo link-* features."
    )
    .unwrap();
    writeln!(
        header,
        " * Buffer sizes are configurable via ZPICO_* environment variables."
    )
    .unwrap();
    writeln!(header, " * DO NOT EDIT — regenerated on every build.").unwrap();
    writeln!(header, " */").unwrap();
    writeln!(header).unwrap();
    writeln!(header, "#ifndef ZENOH_GENERIC_CONFIG_H").unwrap();
    writeln!(header, "#define ZENOH_GENERIC_CONFIG_H").unwrap();
    writeln!(header).unwrap();
    writeln!(header, "// Buffer Sizes").unwrap();
    writeln!(header, "#define Z_FRAG_MAX_SIZE {}", buf.frag_max_size).unwrap();
    writeln!(
        header,
        "#define Z_BATCH_UNICAST_SIZE {}",
        buf.batch_unicast_size
    )
    .unwrap();
    writeln!(
        header,
        "#define Z_BATCH_MULTICAST_SIZE {}",
        buf.batch_multicast_size
    )
    .unwrap();

    const GENERIC_SOCKET_TIMEOUT_MS: u32 = 100;
    const NUTTX_SOCKET_TIMEOUT_MS: u32 = 5000;
    // issue 0906 — this MUST NOT be shorter than the peer's keep-alive cadence,
    // and the peer here is the router ROS ships (RFC-0075).
    //
    // `DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5` announces `lease: 60000` with
    // `keep_alive: 2`, so an idle router speaks every 30 s — and says why:
    // "ROS setting: increase the value to avoid lease expiration at launch time
    // with a large number of Nodes starting all together".
    //
    // zenoh-pico takes `min(peer_lease, Z_TRANSPORT_LEASE)` and closes the
    // session when nothing arrives within it. At 10 s that expired while the
    // router was still 20 s from its next keep-alive — every session, every
    // time, on every platform we ship. Measured: a talker publishing at 1 Hz
    // tore down and reconnected every ~20 s, native Linux identically, and each
    // teardown cost the messages published during it.
    //
    // 60 s matches what ROS announces, so the negotiated `min` is 60 s and the
    // router's 30 s cadence has a 2x margin. The cost is detection latency for
    // a peer that dies without closing its TCP connection: up to 60 s instead
    // of 10 s. That is the same tradeoff every other ROS 2 node on this router
    // already makes, and a peer that dies WITH a socket close is still detected
    // immediately — the lease is the fallback, not the primary signal.
    //
    // Do not lower this without re-reading the router config: the number is a
    // property of the peer, not a taste.
    const Z_TRANSPORT_LEASE_MS: u32 = 60_000;
    let socket_timeout = if target.contains("nuttx") {
        NUTTX_SOCKET_TIMEOUT_MS
    } else {
        GENERIC_SOCKET_TIMEOUT_MS
    };
    writeln!(header, "#define Z_CONFIG_SOCKET_TIMEOUT {}", socket_timeout).unwrap();
    writeln!(header, "#define Z_TRANSPORT_LEASE {}", Z_TRANSPORT_LEASE_MS).unwrap();
    writeln!(header, "#define Z_TRANSPORT_LEASE_EXPIRE_FACTOR 3").unwrap();
    // issue 0959 — the lease task sleeps in chunks of this many ms rather than
    // for a whole keep-alive interval, so `stop_lease_task`'s join does not have
    // to wait out the remainder. Bounds teardown latency at ~1 s instead of
    // `lease / EXPIRE_FACTOR`, which #0906's 60 s lease made 20 s. Costs one
    // wakeup per second on an idle task.
    writeln!(header, "#define Z_TRANSPORT_LEASE_TASK_SLEEP_CHUNK_MS 1000").unwrap();
    writeln!(header, "#define ZP_PERIODIC_SCHEDULER_MAX_TASKS 8").unwrap();
    writeln!(header).unwrap();
    writeln!(header, "// Core Features").unwrap();
    writeln!(header, "#ifndef Z_FEATURE_MULTI_THREAD").unwrap();
    writeln!(header, "#define Z_FEATURE_MULTI_THREAD 0").unwrap();
    writeln!(header, "#endif").unwrap();
    writeln!(header, "#define Z_FEATURE_PUBLICATION 1").unwrap();
    writeln!(header, "#define Z_FEATURE_ADVANCED_PUBLICATION 0").unwrap();
    writeln!(header, "#define Z_FEATURE_SUBSCRIPTION 1").unwrap();
    writeln!(header, "#define Z_FEATURE_ADVANCED_SUBSCRIPTION 0").unwrap();
    writeln!(header, "#define Z_FEATURE_QUERY 1").unwrap();
    writeln!(header, "#define Z_FEATURE_QUERYABLE 1").unwrap();
    writeln!(header, "#define Z_FEATURE_LIVELINESS 1").unwrap();
    writeln!(header, "#define Z_FEATURE_INTEREST 1").unwrap();
    writeln!(header).unwrap();
    writeln!(
        header,
        "// Transport Link Features (from Cargo link-* features)"
    )
    .unwrap();
    writeln!(header, "#define Z_FEATURE_LINK_TCP {}", link.tcp_flag()).unwrap();
    writeln!(
        header,
        "#define Z_FEATURE_LINK_UDP_UNICAST {}",
        link.udp_unicast_flag()
    )
    .unwrap();
    writeln!(
        header,
        "#define Z_FEATURE_LINK_UDP_MULTICAST {}",
        link.udp_multicast_flag()
    )
    .unwrap();
    writeln!(
        header,
        "#define Z_FEATURE_LINK_SERIAL {}",
        link.serial_flag()
    )
    .unwrap();
    writeln!(header, "#define Z_FEATURE_LINK_BLUETOOTH 0").unwrap();
    writeln!(header, "#define Z_FEATURE_LINK_WS 0").unwrap();
    writeln!(header, "#define Z_FEATURE_LINK_SERIAL_USB 0").unwrap();
    writeln!(header, "#define Z_FEATURE_LINK_IVC {}", link.ivc_flag()).unwrap();
    writeln!(
        header,
        "#define Z_FEATURE_LINK_CUSTOM {}",
        link.custom_flag()
    )
    .unwrap();
    // RFC-0080 — CAN / CAN FD.
    writeln!(header, "#define Z_FEATURE_LINK_CAN {}", link.can_flag()).unwrap();
    writeln!(header, "#define Z_FEATURE_LINK_ISOTP {}", link.isotp_flag()).unwrap();
    writeln!(header, "#define Z_FEATURE_LINK_TLS {}", link.tls_flag()).unwrap();
    writeln!(
        header,
        "#define Z_FEATURE_RAWETH_TRANSPORT {}",
        link.raweth_flag()
    )
    .unwrap();
    writeln!(header).unwrap();
    writeln!(header, "// Transport Modes").unwrap();
    writeln!(header, "#define Z_FEATURE_UNICAST_TRANSPORT 1").unwrap();
    // Issue 0682 — off by size decision; see `MULTICAST_TRANSPORT`. Written from
    // that constant so the C library and the Rust session layer cannot disagree
    // about whether peer mode can work.
    writeln!(
        header,
        "#define Z_FEATURE_MULTICAST_TRANSPORT {}",
        multicast_transport_flag()
    )
    .unwrap();
    writeln!(
        header,
        "#define Z_FEATURE_SCOUTING {}",
        multicast_transport_flag()
    )
    .unwrap();
    writeln!(header, "#ifndef Z_FEATURE_SCOUTING_UDP").unwrap();
    writeln!(
        header,
        "#define Z_FEATURE_SCOUTING_UDP {}",
        multicast_transport_flag()
    )
    .unwrap();
    writeln!(header, "#endif").unwrap();
    writeln!(header).unwrap();
    if unstable_api {
        writeln!(header, "#define Z_FEATURE_UNSTABLE_API").unwrap();
    }
    writeln!(header).unwrap();
    writeln!(header, "// Protocol Features").unwrap();
    writeln!(header, "#define Z_FEATURE_FRAGMENTATION 1").unwrap();
    // phase-337 W7.b — was `0` on orin-spe alone (its FSP build had no room for
    // the encoding tables). That board is gone; a future space-constrained
    // platform should turn this off through the RFC-0049 knob ladder like every
    // other `[build.zenoh]` value, not through a board-named bool parameter.
    writeln!(header, "#define Z_FEATURE_ENCODING_VALUES 1").unwrap();
    writeln!(header, "#define Z_FEATURE_TCP_NODELAY 1").unwrap();
    // Same-session loopback (issue 0096): a single `nros::main!` entry registers every
    // node of a launch on ONE zenoh-pico session, so in-process node-to-node delivery —
    // pub→sub AND client query→queryable — depends on these flags. With them 0 the
    // loopback path (src/session/loopback.c) is compiled out and same-session callbacks
    // never fire. Enabled on EVERY target, embedded included: the multi-tier entries
    // (FreeRTOS `run_tiers_entry`, Zephyr `run_tiers`) put all nodes of a launch on one
    // shared session, so intra-image pub→sub is the norm on embedded, not a host-only
    // convenience — and a vanilla zenohd never echoes a put back to the session it came
    // from (issue 0317), so with the flag 0 NO same-session route can deliver, over any
    // path. Remote routes keep working, which disguises the config error as a per-route
    // delivery bug (the rt-eval safety island's cross-tier gate→actuator command on
    // FreeRTOS stayed silent while the identical image on Zephyr, whose cmake forces =1
    // in `zephyr/cmake/nros_rmw_zenoh.cmake`, delivered). This brings the cargo-built
    // embedded lanes (FreeRTOS / NuttX / ThreadX / bare-metal) in line with Zephyr and
    // the host build. The flags are purely additive:
    // a local match sets the write filter to OFF (filtering.c), so the network
    // publication to external subscribers is preserved.
    let local_loopback = 1;
    writeln!(
        header,
        "#define Z_FEATURE_LOCAL_SUBSCRIBER {}",
        local_loopback
    )
    .unwrap();
    writeln!(
        header,
        "#define Z_FEATURE_LOCAL_QUERYABLE {}",
        local_loopback
    )
    .unwrap();
    writeln!(header, "#define Z_FEATURE_SESSION_CHECK 1").unwrap();
    // phase-279 (#145) — Z_FEATURE_BATCHING gates transport-struct FIELDS
    // (_batch_state/_batch_count), so it must be uniform across every TU that
    // includes this header (zenoh-pico + zpico.c — the issue-0135 ABI rule).
    // Flipped here (the shared generated config) by the ZPICO_TX_BATCH knob;
    // stays 0 when the knob is off = byte-identical to today's config.
    writeln!(
        header,
        "#define Z_FEATURE_BATCHING {}",
        if tx_batch { 1 } else { 0 }
    )
    .unwrap();
    // phase-282 (#145) — Z_FEATURE_TX_SPLIT_LOCK also gates struct fields
    // (_wbuf_spare/_mutex_link_tx): same #135 every-TU rule as BATCHING.
    writeln!(
        header,
        "#define Z_FEATURE_TX_SPLIT_LOCK {}",
        if tx_split_lock && tx_batch { 1 } else { 0 }
    )
    .unwrap();
    writeln!(header, "#define Z_FEATURE_BATCH_TX_MUTEX 0").unwrap();
    writeln!(header, "#define Z_FEATURE_BATCH_PEER_MUTEX 0").unwrap();
    writeln!(header, "#define Z_FEATURE_MATCHING 1").unwrap();
    writeln!(header, "#define Z_FEATURE_RX_CACHE 0").unwrap();
    writeln!(header, "#define Z_FEATURE_UNICAST_PEER 0").unwrap();
    // phase-337 W7.b — see `Z_FEATURE_ENCODING_VALUES` above: orin-spe was the
    // only platform that set this to 0 (IVC has no reconnect notion).
    writeln!(header, "#define Z_FEATURE_AUTO_RECONNECT 1").unwrap();
    writeln!(header, "#define Z_FEATURE_MULTICAST_DECLARATIONS 0").unwrap();
    writeln!(header, "#define Z_FEATURE_PERIODIC_TASKS 0").unwrap();
    writeln!(header).unwrap();
    writeln!(header, "#endif /* ZENOH_GENERIC_CONFIG_H */").unwrap();

    header
}

/// Check if we're building for an embedded target.
pub fn is_embedded_target(target: &str) -> bool {
    target.contains("zephyr")
        || target.contains("none")
        || target.contains("nuttx")
        || target.contains("thumbv")
        || target.contains("riscv")
}

pub fn is_plausible_generated_header(header: &str) -> bool {
    header.contains("#ifndef ZPICO_H")
        && header.contains("#define ZPICO_OK")
        && header.contains("typedef void (*ZpicoCallback)")
        && header.contains("int32_t zpico_init(")
}

/// Post-process the generated header to remove duplicate declarations.
pub fn post_process_header(header: &str) -> String {
    let mut result = String::new();
    let mut prev_blank = false;
    let mut skip_until_semicolon = false;
    let mut seen_declarations: HashSet<String> = HashSet::new();
    let mut pending_lines: Vec<String> = Vec::new();

    for line in header.lines() {
        let trimmed = line.trim();

        if line.starts_with("extern ") {
            if !trimmed.ends_with(';') {
                skip_until_semicolon = true;
            }
            continue;
        }

        if skip_until_semicolon {
            if trimmed.ends_with(';') {
                skip_until_semicolon = false;
            }
            continue;
        }

        if trimmed.starts_with("/**") || trimmed.starts_with('*') || trimmed.starts_with("*/") {
            pending_lines.push(line.to_string());
            continue;
        }

        if trimmed.starts_with("typedef ")
            && let Some(name) = extract_typedef_name(trimmed)
        {
            if seen_declarations.contains(&name) {
                pending_lines.clear();
                if !trimmed.ends_with(';') {
                    skip_until_semicolon = true;
                }
                continue;
            }
            seen_declarations.insert(name);
        }

        if (trimmed.starts_with("int32_t ")
            || trimmed.starts_with("void ")
            || trimmed.starts_with("void *")
            || trimmed.starts_with("uint32_t ")
            || trimmed.starts_with("uint64_t ")
            || trimmed.starts_with("bool "))
            && trimmed.contains('(')
            && let Some(name) = extract_function_name(trimmed)
        {
            if seen_declarations.contains(&name) {
                pending_lines.clear();
                if !trimmed.ends_with(';') {
                    skip_until_semicolon = true;
                }
                continue;
            }
            seen_declarations.insert(name);
        }

        for pending in pending_lines.drain(..) {
            result.push_str(&pending);
            result.push('\n');
        }

        let is_blank = trimmed.is_empty();
        if is_blank && prev_blank {
            continue;
        }
        prev_blank = is_blank;

        result.push_str(line);
        result.push('\n');
    }

    result
}

/// Extract function name from a declaration line.
pub fn extract_function_name(line: &str) -> Option<String> {
    let trimmed = line.trim();
    let paren_pos = trimmed.find('(')?;
    let before_paren = &trimmed[..paren_pos];
    let name = before_paren.split_whitespace().last()?;
    Some(name.trim_start_matches('*').to_string())
}

/// Extract typedef name from a declaration line.
pub fn extract_typedef_name(line: &str) -> Option<String> {
    let trimmed = line.trim();

    if let Some(start) = trimmed.find("(*") {
        let after_star = &trimmed[start + 2..];
        if let Some(end) = after_star.find(')') {
            return Some(after_star[..end].to_string());
        }
    }

    if trimmed.ends_with(';') {
        let without_semi = trimmed.trim_end_matches(';').trim();
        return without_semi
            .split_whitespace()
            .last()
            .map(|s| s.to_string());
    }

    None
}

/// Apply an `[arch.*]` profile to a `cc::Build`.
pub fn apply_arch(arch: &manifest::ArchEntry, build: &mut cc::Build, out_dir: &Path) {
    for flag in &arch.cflags {
        build.flag(flag);
    }
    if arch.needs_riscv_compiler {
        detect_riscv_compiler(build);
    }
    if arch.needs_errno_override {
        let errno_dir = out_dir.join("errno-override");
        std::fs::create_dir_all(&errno_dir).unwrap();
        std::fs::write(
            errno_dir.join("errno.h"),
            include_bytes!("../../zpico-sys/c/platform/errno_override.h"),
        )
        .unwrap();
        build.include(&errno_dir);
    }
    if arch.needs_picolibc
        && let Some(sysroot) = get_picolibc_sysroot()
    {
        build.include(sysroot.join("include"));
    }
}

/// Returns `true` when the `[arch.*]` predicates allow the current target triple.
///
/// phase-338 W4 — the predicate now has ONE spelling, in `nros_board_common`
/// beside [`manifest::ArchEntry`] itself. This is kept as the name every zpico
/// caller already uses; it must never regrow a body (CLAUDE.md: add one shared
/// helper, never a second spelling — the class that produced the sizes-header
/// mirror six times).
pub fn arch_matches(arch: &manifest::ArchEntry, target: &str) -> bool {
    nros_board_common::arch_flags::arch_matches(arch, target)
}

/// What a `zenoh-sources.txt` record selects.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ZenohSourceKind {
    /// `dir` — every `.c` under the path, recursively.
    Dir,
    /// `src` — exactly one `.c`.
    File,
}

/// One `dir` / `src` record of `zenoh-sources.txt`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ZenohSourceRecord {
    pub kind: ZenohSourceKind,
    pub group: String,
    pub tree: String,
    pub path: String,
}

/// Parse `packages/rmw/zenoh/zpico-sys/zenoh-sources.txt` — phase-420 W9.
///
/// Returns `(group → condition token, records in file order)`.
///
/// The grammar is deliberately line-oriented and dependency-free, because
/// `zephyr/cmake/nros_rmw_zenoh.cmake` implements the same one with
/// `file(STRINGS)` and a build script here must work from a bare clone under
/// the `--locked` cargo shim. Keep the two implementations answering the same
/// text: `check-zenoh-source-manifest` re-implements it a third time in Python
/// and compares what each lane selects.
pub fn parse_zenoh_sources(
    text: &str,
) -> Result<
    (
        std::collections::BTreeMap<String, String>,
        Vec<ZenohSourceRecord>,
    ),
    String,
> {
    let mut groups = std::collections::BTreeMap::new();
    let mut records = Vec::new();
    for (n, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        match f.as_slice() {
            ["group", name, cond] => {
                if groups
                    .insert((*name).to_string(), (*cond).to_string())
                    .is_some()
                {
                    return Err(format!("line {}: group `{name}` declared twice", n + 1));
                }
            }
            [kind @ ("dir" | "src"), group, tree, path] => records.push(ZenohSourceRecord {
                kind: if *kind == "dir" {
                    ZenohSourceKind::Dir
                } else {
                    ZenohSourceKind::File
                },
                group: (*group).to_string(),
                tree: (*tree).to_string(),
                path: (*path).to_string(),
            }),
            _ => {
                return Err(format!(
                    "line {}: expected `group <name> <condition>`, \
                     `dir <group> <tree> <path>` or `src <group> <tree> <path>`, got `{line}`",
                    n + 1
                ));
            }
        }
    }
    Ok((groups, records))
}

/// Add the zenoh-pico source set this platform compiles to a `cc::Build`.
///
/// phase-420 W9 — THIS FUNCTION OWNS NO SOURCE LIST. The nine directory globs
/// that used to live here are read from `zpico-sys/zenoh-sources.txt`, and
/// `zephyr/cmake/nros_rmw_zenoh.cmake` reads the same file. There were two
/// copies of that selection, one per language lane, and nothing checked that
/// they agreed — the class issue 1068 fixed for micro-XRCE next door. Gate:
/// `just check zenoh-source-manifest`.
///
/// `platform` is the resolved `nros-platform.toml` platform name and `link` its
/// link features; between them they answer the manifest's condition tokens.
pub fn add_zenoh_pico_core_sources(
    build: &mut cc::Build,
    zenoh_pico_src: &Path,
    manifest_path: &Path,
    platform: &str,
    link: &LinkFeatures,
) {
    // A manifest edit must rebuild the C. Nothing else here watches this file.
    println!("cargo:rerun-if-changed={}", manifest_path.display());
    let text = std::fs::read_to_string(manifest_path).unwrap_or_else(|e| {
        panic!(
            "nros-zpico-build: cannot read the shared zenoh-pico source manifest {} ({e}). \
             It is tracked in-repo — a missing one means the checkout is incomplete.",
            manifest_path.display()
        )
    });
    let (groups, records) = parse_zenoh_sources(&text)
        .unwrap_or_else(|e| panic!("nros-zpico-build: {}: {e}", manifest_path.display()));

    // NROS-ZENOH-CONDITIONS-BEGIN — the boolean this lane supplies for each
    // condition token the manifest uses. `check-zenoh-source-manifest` asserts
    // the cmake lane answers exactly this token set, and that it is exactly the
    // set the manifest uses; the manifest, not this match, decides which
    // sources a token covers.
    let is_zephyr = platform == "zephyr";
    let condition = |token: &str| -> bool {
        match token {
            "always" => true,
            // FALSE by construction today, and written as the platform test
            // rather than a literal `false` so it stays true of the code
            // rather than of today's routing: Zephyr declares `compiled_by =
            // "platform"` in `nros-platform.toml`, so the cargo lane never
            // compiles the vendored tree for it (issue 0541 — when it briefly
            // did, it died on `version.h: No such file or directory`, a header
            // only the Zephyr build generates). The cmake lane answers TRUE.
            "zephyr" => is_zephyr,
            // RFC-0083 — Zephyr AND the ISO-TP link is wanted.
            "zephyr_isotp" => is_zephyr && link.isotp,
            other => panic!(
                "nros-zpico-build: {} uses condition token `{other}`, which this lane does \
                 not answer. Add an arm here AND a `set(_zenoh_cond_{other} …)` in \
                 zephyr/cmake/nros_rmw_zenoh.cmake — a token only one lane answers is the \
                 same defect one conditional over.",
                manifest_path.display()
            ),
        }
    };
    // NROS-ZENOH-CONDITIONS-END

    let tree_root = |tree: &str| -> PathBuf {
        match tree {
            "zenoh_pico" => zenoh_pico_src.join("src"),
            other => panic!(
                "nros-zpico-build: {} names unknown source tree `{other}`",
                manifest_path.display()
            ),
        }
    };

    let mut added = 0usize;
    for record in &records {
        let token = groups.get(&record.group).unwrap_or_else(|| {
            panic!(
                "nros-zpico-build: {}: `{}` names group `{}`, which no `group` line declares",
                manifest_path.display(),
                record.path,
                record.group
            )
        });
        if !condition(token) {
            continue;
        }
        let path = tree_root(&record.tree).join(&record.path);
        match record.kind {
            ZenohSourceKind::Dir => {
                assert!(
                    path.is_dir(),
                    "nros-zpico-build: {} lists directory `{}/{}`, which is not a directory \
                     at {}. An upstream bump that renamed or removed it needs the manifest \
                     updated, not this build script.",
                    manifest_path.display(),
                    record.tree,
                    record.path,
                    path.display()
                );
                added += add_c_sources_recursive(build, &path);
            }
            ZenohSourceKind::File => {
                assert!(
                    path.is_file(),
                    "nros-zpico-build: {} lists source `{}/{}`, which does not exist at {}",
                    manifest_path.display(),
                    record.tree,
                    record.path,
                    path.display()
                );
                build.file(&path);
                added += 1;
            }
        }
    }
    assert!(
        added > 0,
        "nros-zpico-build: {} selected no sources — manifest or condition drift",
        manifest_path.display()
    );
}

/// Recursively collect all .c files from a directory and add them to a
/// `cc::Build`; returns how many were added.
///
/// phase-420 W9 — reached only through [`add_zenoh_pico_core_sources`]'s `dir`
/// records now. It expands a rule the manifest states; it does not decide one.
pub fn add_c_sources_recursive(build: &mut cc::Build, dir: &Path) -> usize {
    if !dir.exists() {
        return 0;
    }
    let mut added = 0usize;
    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .filter_map(Result::ok)
        .collect();
    entries.sort_by_key(|e| e.file_name());
    for entry in entries {
        let path = entry.path();
        if path.is_dir() {
            added += add_c_sources_recursive(build, &path);
        } else if path.extension().is_some_and(|ext| ext == "c") {
            build.file(&path);
            added += 1;
        }
    }
    added
}

/// RISC-V bare-metal GCC binary names tried, in preference order, wherever
/// zpico compiles C for a riscv target. `riscv-none-elf-gcc` is the xPack
/// multilib toolchain `nros setup` provisions (rv32imc/ilp32 capable); its
/// omission (issue 0399) meant a host provisioned exactly as documented fell
/// through to cc-rs's guessed `riscv32-unknown-elf-gcc` and failed to build.
/// One list so the detect / sysroot / picolibc-specs probes can never diverge.
pub const RISCV_GCC_CANDIDATES: &[&str] = &[
    "riscv64-unknown-elf-gcc",
    "riscv32-esp-elf-gcc",
    "riscv-none-elf-gcc",
];

/// Detect and set the RISC-V cross-compiler for cc::Build.
pub fn detect_riscv_compiler(build: &mut cc::Build) {
    // issue 0657 — for a riscv64 target, defer to the SHARED resolver. This
    // list is ordered for the esp32 (riscv32) case, where the esp toolchain
    // must win; on riscv64 it put Ubuntu's `riscv64-unknown-elf-gcc` ahead of
    // the xPack build that `nros setup` provisions, so a host with both
    // compiled the zenoh C shim with one toolchain and the ThreadX board with
    // the other — two libcs in one image, agreeing about nothing.
    let target = std::env::var("TARGET").unwrap_or_default();
    if target.starts_with("riscv64")
        && let Some(gcc) = nros_build_paths::riscv64::tool("gcc")
    {
        build.compiler(gcc);
        return;
    }
    for cc_name in RISCV_GCC_CANDIDATES {
        if Command::new(cc_name).arg("--version").output().is_ok() {
            build.compiler(cc_name);
            return;
        }
    }

    // issue 0399, second half — when NOTHING matched, this returned quietly and
    // left cc-rs to derive a name from the target triple
    // (`riscv32-unknown-elf-gcc`). No board installs that name, so the build
    // died naming a compiler nobody configured, with nothing pointing at the
    // real problem — a toolchain that was never provisioned:
    //
    //     error occurred in cc-rs: failed to find tool "riscv32-unknown-elf-gcc"
    //
    // Fail here instead, naming the remedy. A host where cc-rs's own default
    // DOES exist was already working, so it keeps working.
    let target = std::env::var("TARGET").unwrap_or_default();
    let cc_rs_default = if target.starts_with("riscv64") {
        "riscv64-unknown-elf-gcc"
    } else {
        "riscv32-unknown-elf-gcc"
    };
    if Command::new(cc_rs_default)
        .arg("--version")
        .output()
        .is_ok()
    {
        return;
    }

    panic!(
        "no RISC-V cross-compiler found for TARGET={target}. Tried \
         {RISCV_GCC_CANDIDATES:?} and cc-rs's default `{cc_rs_default}`. Install the \
         toolchain this repo provisions:\n\
         \n    nros setup --tool riscv-none-elf-gcc\n\
         \n(or `nros setup <board>` for the board you are building)"
    );
}

/// Get the picolibc sysroot path for RISC-V.
pub fn get_picolibc_sysroot() -> Option<PathBuf> {
    for cc_name in RISCV_GCC_CANDIDATES {
        if let Ok(output) = Command::new(cc_name)
            .args([
                "-march=rv32imc",
                "-mabi=ilp32",
                "--specs=picolibc.specs",
                "-print-sysroot",
            ])
            .output()
            && output.status.success()
        {
            let sysroot = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if !sysroot.is_empty() {
                let path = PathBuf::from(&sysroot);
                if path.join("include").exists() {
                    return Some(path);
                }
            }
        }
    }
    let fallback = PathBuf::from("/usr/lib/picolibc/riscv64-unknown-elf");
    if fallback.join("include").exists() {
        return Some(fallback);
    }
    None
}

/// Check if the RISC-V GCC supports picolibc specs.
pub fn has_picolibc_specs() -> bool {
    for cc in RISCV_GCC_CANDIDATES {
        if let Ok(status) = Command::new(cc)
            .args([
                "-march=rv32imc",
                "-mabi=ilp32",
                "--specs=picolibc.specs",
                "-E",
                "-x",
                "c",
                "/dev/null",
                "-o",
                "/dev/null",
            ])
            .status()
            && status.success()
        {
            return true;
        }
    }
    false
}

/// Read the size of a symbol from a static library using llvm-nm or nm.
pub fn read_symbol_size(
    archive: &Path,
    symbol: &str,
    rustc_sysroot: Option<&str>,
    host: &str,
) -> usize {
    let sysroot = rustc_sysroot.map(ToOwned::to_owned).or_else(|| {
        Command::new("rustc")
            .args(["--print", "sysroot"])
            .output()
            .ok()
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    });

    let llvm_nm_candidates = [
        sysroot
            .as_ref()
            .map(|s| {
                PathBuf::from(s)
                    .join("lib/rustlib")
                    .join(host)
                    .join("bin/llvm-nm")
            })
            .unwrap_or_default(),
        PathBuf::from("llvm-nm"),
        PathBuf::from("nm"),
    ];

    for nm in &llvm_nm_candidates {
        if let Ok(output) = Command::new(nm)
            .args(["--print-size", "--defined-only"])
            .arg(archive)
            .output()
            && output.status.success()
        {
            let stdout = String::from_utf8_lossy(&output.stdout);
            for line in stdout.lines() {
                if line.contains(symbol) {
                    let parts: Vec<&str> = line.split_whitespace().collect();
                    if parts.len() >= 2
                        && let Ok(size) = usize::from_str_radix(parts[1], 16)
                        && size > 0
                    {
                        return size;
                    }
                }
            }
        }
    }

    eprintln!(
        "cargo:warning=Could not determine size of {} from {}, using fallback 16",
        symbol,
        archive.display()
    );
    16
}

/// Generate pkg-config `.pc` file contents for mbedTLS.
pub fn mbedtls_pc_files(lib_dir: &str) -> [(&'static str, String); 3] {
    fn pc(name: &str, libs: &str, requires: &str, lib_dir: &str) -> String {
        format!(
            "prefix=/usr\n\
             libdir={lib_dir}\n\
             includedir=/usr/include\n\n\
             Name: {name}\n\
             Description: mbed TLS - {name}\n\
             Version: 2.28.0\n\
             Libs: -L${{libdir}} {libs}\n\
             Cflags: -I${{includedir}}\n\
             Requires: {requires}\n"
        )
    }

    [
        ("mbedcrypto", pc("mbedcrypto", "-lmbedcrypto", "", lib_dir)),
        (
            "mbedx509",
            pc("mbedx509", "-lmbedx509", "mbedcrypto", lib_dir),
        ),
        ("mbedtls", pc("mbedtls", "-lmbedtls", "mbedx509", lib_dir)),
    ]
}

/// Generate zenoh-pico version header contents.
pub fn embedded_version_header(version: &str, template: &str) -> String {
    let version = version.trim();
    let parts: Vec<&str> = version.split('.').collect();
    let major = parts.first().unwrap_or(&"0");
    let minor = parts.get(1).unwrap_or(&"0");
    let patch = parts.get(2).unwrap_or(&"0");

    template
        .replace("@ZENOH_PICO@", version)
        .replace("@ZENOH_PICO_MAJOR@", major)
        .replace("@ZENOH_PICO_MINOR@", minor)
        .replace("@ZENOH_PICO_PATCH@", patch)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extracts_function_names_from_c_declarations() {
        assert_eq!(
            extract_function_name("int32_t zpico_init(const char *locator);").as_deref(),
            Some("zpico_init")
        );
        assert_eq!(
            extract_function_name("void *zpico_loan(size_t len);").as_deref(),
            Some("zpico_loan")
        );
    }

    #[test]
    fn extracts_typedef_names() {
        assert_eq!(
            extract_typedef_name("typedef void (*ZpicoCallback)(void *ctx);").as_deref(),
            Some("ZpicoCallback")
        );
        assert_eq!(
            extract_typedef_name("typedef struct zpico_session ZpicoSession;").as_deref(),
            Some("ZpicoSession")
        );
    }

    #[test]
    fn post_process_header_removes_externs_and_duplicate_declarations() {
        let header = "\
extern int hidden;\n\
/** first */\n\
typedef int ZpicoFoo;\n\
/** duplicate */\n\
typedef int ZpicoFoo;\n\
\n\
\n\
int32_t zpico_init(void);\n\
int32_t zpico_init(void);\n";
        let processed = post_process_header(header);

        assert!(!processed.contains("extern int hidden"));
        assert!(processed.contains("/** first */\ntypedef int ZpicoFoo;"));
        assert!(!processed.contains("duplicate"));
        assert_eq!(processed.matches("zpico_init").count(), 1);
        assert!(!processed.contains("\n\n\n"));
    }

    #[test]
    fn shim_consts_are_generated_from_config() {
        let cfg = ShimConfig {
            max_publishers: 1,
            max_subscribers: 2,
            max_queryables: 3,
            queryable_table_declared: true,
            max_liveliness: 4,
            max_pending_gets: 5,
            max_sessions: 9,
            get_reply_buf_size: 6,
            get_poll_interval_ms: 7,
            tx_batch: false,
            tx_batch_flush_ms: 50,
            read_task_priority: 11,
            lease_task_priority: 12,
        };

        let body = cfg.rust_consts();
        assert!(body.contains("ZPICO_MAX_PUBLISHERS: usize = 1;"));
        assert!(body.contains("ZPICO_MAX_PENDING_GETS: usize = 5;"));
        assert!(body.contains("ZPICO_MAX_SESSIONS: usize = 9;"));
        // phase-392 W5.e — the shim's exhaustion message branches on this, so
        // it has to arrive with the size it describes.
        assert!(body.contains("ZPICO_QUERYABLE_TABLE_DECLARED: bool = true;"));
        assert!(!body.contains("get_reply_buf_size"));
    }

    /// issue 0626 / 0460 — the transport-task priorities must reach the C shim
    /// from the RUST lane too.
    ///
    /// `nros_rmw_zenoh.cmake` passes them as compile definitions on the C lane,
    /// and `zpico.c` `#define`s its own 16/16 fallback, so a Rust lane that
    /// omitted them did not "leave the priority unset" — it silently took the C
    /// default while Kconfig said otherwise, on the one platform where the
    /// setting exists at all. That is issue 0460's shape, and it is why these
    /// are emitted unconditionally rather than under a feature test.
    #[test]
    fn transport_task_priorities_reach_the_c_shim() {
        for tx_batch in [false, true] {
            let cfg = ShimConfig {
                max_publishers: 1,
                max_subscribers: 2,
                max_queryables: 3,
                queryable_table_declared: false,
                max_liveliness: 4,
                max_pending_gets: 5,
                max_sessions: 1,
                get_reply_buf_size: 6,
                get_poll_interval_ms: 7,
                tx_batch,
                tx_batch_flush_ms: 50,
                read_task_priority: 11,
                lease_task_priority: 12,
            };
            let defines = cfg.defines();
            let get = |name: &str| {
                defines
                    .iter()
                    .find(|(k, _)| *k == name)
                    .unwrap_or_else(|| panic!("{name} not defined (tx_batch={tx_batch})"))
                    .1
                    .clone()
            };
            assert_eq!(get("ZPICO_READ_TASK_PRIORITY"), "11");
            assert_eq!(get("ZPICO_LEASE_TASK_PRIORITY"), "12");
            // The knobs that were already correct stay correct in both states —
            // the point of the loop is that `tx_batch` gates only its own pair.
            assert_eq!(get("ZPICO_MAX_QUERYABLES"), "3");
            // …and the other value of the W5.e flag really is emitted.
            assert!(
                cfg.rust_consts()
                    .contains("ZPICO_QUERYABLE_TABLE_DECLARED: bool = false;")
            );
            assert_eq!(
                defines.iter().any(|(k, _)| *k == "ZPICO_TX_BATCH"),
                tx_batch
            );
        }
    }

    #[test]
    fn config_header_uses_resolved_link_and_platform_values() {
        let link = LinkFeatures {
            tcp: true,
            udp_unicast: false,
            udp_multicast: false,
            serial: true,
            raweth: false,
            tls: true,
            ivc: false,
            can: false,
            isotp: false,
            custom: true,
        };
        let buf = ZenohBufferConfig {
            frag_max_size: 4096,
            batch_unicast_size: 2048,
            batch_multicast_size: 1024,
        };

        let header = config_header(&link, &buf, "armv7a-nuttx-eabihf", true, false, false);

        assert!(header.contains("#define Z_FRAG_MAX_SIZE 4096"));
        assert!(header.contains("#define Z_CONFIG_SOCKET_TIMEOUT 5000"));
        assert!(header.contains("#define Z_FEATURE_LINK_TCP 1"));
        assert!(header.contains("#define Z_FEATURE_LINK_UDP_UNICAST 0"));
        assert!(header.contains("#define Z_FEATURE_LINK_TLS 1"));
        // RFC-0080 — the CAN link's define is emitted like every other link's.
        // Asserted because the field was added to `LinkFeatures` without a test
        // that reads it back: the two literals here had to gain `can` to keep
        // compiling, and a field that only ever appears as `false` in a struct
        // literal is not covered by anything.
        assert!(header.contains("#define Z_FEATURE_LINK_CAN 0"));
        assert!(header.contains("#define Z_FEATURE_LINK_ISOTP 0"));
        assert!(header.contains("#define Z_FEATURE_LINK_CUSTOM 1"));
        assert!(header.contains("#define Z_FEATURE_UNSTABLE_API"));
        // phase-337 W7.b — both were 0 only under the `orin_spe` flag this
        // signature used to carry; with the board gone they are unconditional.
        assert!(header.contains("#define Z_FEATURE_ENCODING_VALUES 1"));
        assert!(header.contains("#define Z_FEATURE_AUTO_RECONNECT 1"));
        // issue 0096 — same-session loopback is ON everywhere, embedded included:
        // the multi-tier entries share one session across all tiers, and zenohd
        // never echoes a put back to its source session (issue 0317), so without
        // the loopback no intra-image route can deliver (the silent gate→actuator
        // drop on FreeRTOS).
        assert!(header.contains("#define Z_FEATURE_LOCAL_SUBSCRIBER 1"));
        assert!(header.contains("#define Z_FEATURE_LOCAL_QUERYABLE 1"));
    }

    #[test]
    fn host_build_enables_same_session_loopback() {
        // issue 0096 — the HOST/native build compiles in zenoh-pico's same-session
        // loopback so two nodes of one `nros::main!` entry can talk in-process.
        let link = LinkFeatures {
            tcp: true,
            udp_unicast: false,
            udp_multicast: false,
            serial: false,
            raweth: false,
            tls: false,
            ivc: false,
            can: false,
            isotp: false,
            custom: false,
        };
        let buf = ZenohBufferConfig {
            frag_max_size: 4096,
            batch_unicast_size: 2048,
            batch_multicast_size: 1024,
        };

        let header = config_header(&link, &buf, "x86_64-unknown-linux-gnu", false, false, false);

        assert!(header.contains("#define Z_FEATURE_LOCAL_SUBSCRIBER 1"));
        assert!(header.contains("#define Z_FEATURE_LOCAL_QUERYABLE 1"));
    }

    #[test]
    fn mbedtls_pc_files_point_at_requested_lib_dir() {
        let files = mbedtls_pc_files("/opt/lib");

        assert_eq!(files[0].0, "mbedcrypto");
        assert!(files[0].1.contains("libdir=/opt/lib"));
        assert!(files[1].1.contains("Requires: mbedcrypto"));
        assert!(files[2].1.contains("Requires: mbedx509"));
    }

    #[test]
    fn version_header_replaces_template_tokens() {
        let template = "@ZENOH_PICO@ @ZENOH_PICO_MAJOR@ @ZENOH_PICO_MINOR@ @ZENOH_PICO_PATCH@";
        assert_eq!(embedded_version_header("1.2.3\n", template), "1.2.3 1 2 3");
    }

    #[test]
    fn embedded_target_detection_matches_supported_families() {
        assert!(is_embedded_target("thumbv7em-none-eabihf"));
        assert!(is_embedded_target("riscv32imc-unknown-none-elf"));
        assert!(is_embedded_target("armv7a-nuttx-eabihf"));
        assert!(!is_embedded_target("x86_64-unknown-linux-gnu"));
    }

    // phase-420 W9 — the shared `zenoh-sources.txt` grammar. Three
    // implementations answer this text (here, `file(STRINGS)` in
    // `zephyr/cmake/nros_rmw_zenoh.cmake`, and Python in
    // `scripts/check-zenoh-source-manifest.py`), so it stays boringly simple
    // and each one is tested where it lives.
    #[test]
    fn zenoh_source_manifest_grammar() {
        let (groups, records) = parse_zenoh_sources(
            "# comment\n\
             group core    always\n\
             group zeph    zephyr\n\
             \n\
             dir core zenoh_pico api\n\
             src zeph zenoh_pico system/zephyr/network.c  # trailing\n",
        )
        .unwrap();
        assert_eq!(groups["core"], "always");
        assert_eq!(groups["zeph"], "zephyr");
        assert_eq!(records.len(), 2);
        assert_eq!(records[0].kind, ZenohSourceKind::Dir);
        assert_eq!(records[0].path, "api");
        assert_eq!(records[1].kind, ZenohSourceKind::File);
        assert_eq!(records[1].path, "system/zephyr/network.c");
    }

    #[test]
    fn zenoh_source_manifest_rejects_malformed_records() {
        for broken in [
            "group core\n",
            "dir core zenoh_pico\n",
            "glob core zenoh_pico api\n",
            "banana\n",
        ] {
            assert!(
                parse_zenoh_sources(broken).is_err(),
                "parser accepted `{broken}`"
            );
        }
        assert!(parse_zenoh_sources("group core always\ngroup core zephyr\n").is_err());
    }
}
