fn main() {
    // issue 0682 — the peer-mode build input (`just test-zpico-peer`).
    println!("cargo:rerun-if-env-changed=ZPICO_MULTICAST_TRANSPORT");
    println!("cargo:rerun-if-env-changed=NROS_SUBSCRIBER_BUFFER_SIZE");
    println!("cargo:rerun-if-env-changed=ZPICO_SERVICE_BUFFER_SIZE");
    println!("cargo:rerun-if-env-changed=NROS_SERVICE_TIMEOUT_MS");
    println!("cargo:rerun-if-env-changed=NROS_KEYEXPR_STRING_SIZE");
    println!("cargo:rerun-if-env-changed=ZPICO_SUBSCRIBER_RING_DEPTH");
    println!("cargo:rerun-if-env-changed=ZPICO_SUBSCRIBER_LARGE_SIZE");
    println!("cargo:rerun-if-env-changed=ZPICO_SUBSCRIBER_SIZE_THRESHOLD");
    println!("cargo:rerun-if-env-changed=ZPICO_MAX_LARGE_SUBSCRIBERS");
    // issue 1122 — WATCH what we READ. Consuming a declared fact without
    // declaring it is what let the queryable tables go stale (the note on
    // `resolve_queryable_default` one crate over): an image that gains or
    // loses a subscription keeps its previously-sized pool until something
    // else forces a rebuild, and the sizing then reads as applied while being
    // stale.
    println!("cargo:rerun-if-env-changed=NROS_DECLARED_LARGE_SUBSCRIBERS");
    println!("cargo:rerun-if-env-changed=NROS_DECLARED_SUBSCRIBER_BUFFER_SIZE");
    println!("cargo:rerun-if-env-changed=NROS_DECLARED_SUBSCRIBER_LARGE_SIZE");
    println!("cargo:rerun-if-env-changed=NROS_EXECUTOR_MAX_NODES");
    println!("cargo:rerun-if-env-changed=ZPICO_PUBLISHER_TX_BUFFER_SIZE");

    // Phase 214.C.3 — default coordinated with
    // `packages/core/nros-node/build.rs::NROS_SUBSCRIPTION_BUFFER_SIZE`
    // (also 1024). If you change one, change the other — they share the
    // wire-format expectation. Both can be overridden independently via
    // their respective env vars.
    // issue 1199 — the derived SMALL class, on the same DECLARED road as the
    // large count below. The derivation publishes it only when something
    // received actually fits under the ceiling, so an absent variable is "no
    // answer" and the crate default stands.
    let sub_size: usize = env_usize_rung(
        "NROS_SUBSCRIBER_BUFFER_SIZE",
        declared_usize("NROS_DECLARED_SUBSCRIBER_BUFFER_SIZE"),
        1024,
    );
    let svc_size: usize = env_usize("ZPICO_SERVICE_BUFFER_SIZE", 1024);
    // Phase 160.C.2 — bumped 10_000 → 30_000. The original 10 s default
    // was too short for slow zenoh-pico flushes on Zephyr/NSOS where
    // each publish/query can take ~2.5 s under Z_FEATURE_INTEREST=1. An
    // action `get_result` query sent while the server is still running a
    // feedback loop (11 publishes × ~2.5 s each = ~28 s before
    // `complete_goal` fires) expires the internal query timer well
    // before the server reaches its `try_handle_get_result` handler.
    // Bumping to 30 s covers the common slow-Zephyr action window; fast
    // services on POSIX still return in milliseconds so the wider cap
    // only matters when something is genuinely slow.
    // phase-400 W6 — the `[knobs.zenoh.limits]` rungs. `None` when no lane named
    // a platform, and the builtins below then stand.
    let limits = nros_platform_config::platform_config::BuildRungs::from_build_env()
        .map(|r| r.zenoh_limit_rungs())
        .unwrap_or_default();
    // NOT a ladder knob, deliberately: this value is read HERE and in
    // `nros-build-helpers`'s C emitter, because two artifacts embed it (a Rust
    // const and a C define). `check-knob-single-reader` allows a migrated knob
    // exactly one reader, and that invariant is what stops the pair drifting —
    // so migrating this one needs the two readers to share a single resolution
    // point first, which is a change to where the value is EMITTED, not to the
    // ladder.
    let service_timeout_ms: usize = env_usize("NROS_SERVICE_TIMEOUT_MS", 30_000);
    let keyexpr_string_size: usize =
        env_usize_rung("NROS_KEYEXPR_STRING_SIZE", limits.keyexpr_string_size, 256);
    // Phase 124.D.3.c — SPSC ring depth per subscriber. Default 4
    // keeps the static-RAM bump small (4 × SUBSCRIBER_BUFFER_SIZE
    // per subscriber); raise for burst-heavy topics. Must be ≥ 1.
    let ring_depth: usize = env_usize_min(
        "ZPICO_SUBSCRIBER_RING_DEPTH",
        limits.subscriber_ring_depth.unwrap_or(4),
        1,
    );
    // Phase 231 (RFC-0038) — size-class receive buffers. `SUBSCRIBER_BUFFER_SIZE`
    // above is the `small` class slot size; the `large` class is for big
    // messages (images, point clouds). A subscription routes to `large` when its
    // `rx_buffer_hint` exceeds the threshold. `large` is capped at a small count
    // so the big slots don't multiply across every subscriber.
    // issue 1199 — the derived LARGE class size. Published only when the large
    // COUNT is non-zero: a size for a class with no blocks would be inventing a
    // number, which is the rule `_nros_bounds_publish_payload_classes` states
    // and `DERIVED_PAYLOAD_ENV_KEYS` repeats for the leaf road.
    let large_size: usize = env_usize_rung(
        "ZPICO_SUBSCRIBER_LARGE_SIZE",
        declared_usize("NROS_DECLARED_SUBSCRIBER_LARGE_SIZE"),
        16384,
    );
    let size_threshold: usize = env_usize("ZPICO_SUBSCRIBER_SIZE_THRESHOLD", 2048);
    // Phase 403 W4 — the count of LARGE-class blocks, and 0 is legal.
    //
    // This carried issue 0827's floor of 1 until W4, on the stated ground that
    // "the lookup path indexes the pool unconditionally". That is true of the
    // other two floored knobs and it is NOT true here: `alloc_payload_block`
    // tests `idx >= MAX_LARGE_SUBSCRIBERS` BEFORE it indexes `LARGE_PAYLOADS`,
    // so a zero-length pool returns `None` and is never subscripted. The floor
    // was therefore charging every image `RING_DEPTH * LARGE_SIZE` bytes
    // (65,536 at the defaults) for a class it may never route a single
    // subscription into -- which is the shape of the waste 0827 exists to
    // remove, kept alive by 0827's own guard.
    //
    // Zero is a claim, not a shrug: it says this image's types all fit the
    // small class. `alloc_payload_block` refuses a hint that no class can hold,
    // so getting it wrong fails at `create_subscription` rather than dropping
    // every sample at the transport.
    //
    // issue 1122 — the DEFAULT comes from the declaration when cmake made one.
    // `nros_derive_message_bound_knobs()` derives this correctly on every lane
    // and, before 1122, only the Zephyr lane could deliver it; the number was
    // computed, written to disk and discarded everywhere else. It now travels
    // as `NROS_DECLARED_LARGE_SUBSCRIBERS`, on the same lane-independent
    // carrier `NROS_DECLARED_SERVICE_SERVERS` already uses.
    //
    // It is the DEFAULT and not the value, so rung 1 of the ladder survives: a
    // consumer who names `ZPICO_MAX_LARGE_SUBSCRIBERS` still wins. Setting the
    // knob itself in the child environment would have made the named override
    // unreachable, silently.
    let max_large: usize = env_usize_rung(
        "ZPICO_MAX_LARGE_SUBSCRIBERS",
        declared_usize("NROS_DECLARED_LARGE_SUBSCRIBERS"),
        2,
    );
    // Phase 268 — per-session per-node NN liveliness token cap. One zenoh
    // session hosts at most the executor's node cap of graph nodes, so this
    // tracks `nros-node`'s `NROS_EXECUTOR_MAX_NODES` (default 4); keep them in
    // sync — set the same env var for both. `.max(1)` so a session always has
    // room for its own primary node.
    let max_nodes: usize = env_usize_min("NROS_EXECUTOR_MAX_NODES", 4, 1);
    // Issue 0813 — per-publisher TX arena capacity for the zero-copy loan path
    // (`SlotLending`). This was a bare `const` in `shim/publisher.rs`, so its
    // 1 KiB ceiling was neither raisable by a consumer nor visible to
    // `scripts/gen-pool-inventory.py`. It is the publisher-side twin of
    // `NROS_SUBSCRIBER_BUFFER_SIZE` and shares its default. The arena is
    // per-publisher, so the cost is `ZPICO_MAX_PUBLISHERS` × this — priced in
    // the inventory via the `nros-pool:` annotation beside `LendArena`.
    let publisher_tx_size: usize = env_usize("ZPICO_PUBLISHER_TX_BUFFER_SIZE", 1024);

    let out_dir = std::env::var("OUT_DIR").unwrap();
    let path = std::path::Path::new(&out_dir).join("buffer_config.rs");
    std::fs::write(
        &path,
        format!(
            "/// Subscriber buffer size (set via NROS_SUBSCRIBER_BUFFER_SIZE, default 1024).\n\
             pub const SUBSCRIBER_BUFFER_SIZE: usize = {sub_size};\n\
             /// Service request buffer size (set via ZPICO_SERVICE_BUFFER_SIZE, default 1024).\n\
             pub const SERVICE_BUFFER_SIZE: usize = {svc_size};\n\
             /// Default service client RPC timeout in milliseconds\n\
             /// (set via NROS_SERVICE_TIMEOUT_MS, default 30000).\n\
             pub const SERVICE_DEFAULT_TIMEOUT_MS: u32 = {service_timeout_ms};\n\
             /// Maximum key expression string size for topic/service names\n\
             /// (set via NROS_KEYEXPR_STRING_SIZE, default 256).\n\
             pub const KEYEXPR_STRING_SIZE: usize = {keyexpr_string_size};\n\
             /// Key expression buffer size (KEYEXPR_STRING_SIZE + 1 for null terminator).\n\
             pub const KEYEXPR_BUFFER_SIZE: usize = {keyexpr_buf_size};\n\
             /// Phase 124.D.3.c — per-subscriber SPSC ring depth\n\
             /// (set via ZPICO_SUBSCRIBER_RING_DEPTH, default 4).\n\
             pub const SUBSCRIBER_RING_DEPTH: usize = {ring_depth};\n\
             /// Phase 231 (RFC-0038) — `large` size-class slot size\n\
             /// (set via ZPICO_SUBSCRIBER_LARGE_SIZE, default 16384).\n\
             pub const SUBSCRIBER_LARGE_SIZE: usize = {large_size};\n\
             /// Phase 231 — rx_buffer_hint above this routes to the `large` class\n\
             /// (set via ZPICO_SUBSCRIBER_SIZE_THRESHOLD, default 2048).\n\
             pub const SUBSCRIBER_SIZE_THRESHOLD: usize = {size_threshold};\n\
             /// Phase 231 — max concurrent `large`-class subscribers\n\
             /// (set via ZPICO_MAX_LARGE_SUBSCRIBERS, default 2).\n\
             pub const MAX_LARGE_SUBSCRIBERS: usize = {max_large};\n\
             /// Phase 268 — per-session per-node NN liveliness token cap, tracking\n\
             /// `nros-node`'s NROS_EXECUTOR_MAX_NODES (default 4): one session hosts\n\
             /// at most that many graph nodes.\n\
             pub const MAX_PER_NODE_LIVELINESS: usize = {max_nodes};\n\
             /// Issue 0813 — per-publisher TX arena capacity for the zero-copy\n\
             /// loan path (set via ZPICO_PUBLISHER_TX_BUFFER_SIZE, default 1024).\n\
             pub const PUBLISHER_TX_BUFFER_SIZE: usize = {publisher_tx_size};\n",
            keyexpr_buf_size = keyexpr_string_size + 1,
        ),
    )
    .unwrap();
}

/// The Kconfig option each knob is resolved from on Zephyr. Only the two the
/// cmake side forwards (`_nros_resolve_knob` in `nros_cargo_build.cmake`) have
/// a row; the rest are env-or-default as before. See issue 0460 and the twin
/// table in `nros-zpico-build`'s runner — a Zephyr RUST image never inherits
/// the cmake `set(ENV{...})` exports, so without this a Kconfig'd buffer size
/// reached the C lane and silently did not reach this crate.
const KCONFIG_KNOBS: &[(&str, &str)] = &[
    (
        "NROS_SUBSCRIBER_BUFFER_SIZE",
        "CONFIG_NROS_SUBSCRIBER_BUFFER_SIZE",
    ),
    (
        "ZPICO_SERVICE_BUFFER_SIZE",
        "CONFIG_NROS_SERVICE_BUFFER_SIZE",
    ),
];

/// issue 0827 — a floored knob must REFUSE a value below its floor, never
/// round it up.
///
/// These pools cannot be zero-length: the lookup paths index them
/// unconditionally, which is what the `.max(1)` this replaces was protecting.
/// But `.max(1)` protected it by SILENTLY substituting 1, so a knob of 0 built
/// a pool and reported nothing.
///
/// Phase 403 W4 — `ZPICO_MAX_LARGE_SUBSCRIBERS` was the third member and is no
/// longer floored. It never met the premise: `alloc_payload_block` bounds-checks
/// the class index BEFORE subscripting `LARGE_PAYLOADS`, so a zero-length large
/// pool returns `None` rather than indexing out of range. Its floor was
/// reserving 65,536 bytes at the defaults for a class an image whose types all
/// fit the small class never routes into. Read that as the rule this doc
/// already states, applied to itself: a floor is only honest where the lookup
/// really cannot refuse the entity kind first, and here it can.
///
/// That matters now because 0827's fix derives these knobs from the resolved
/// model: an image with no subscriptions would ask for 0, get 1, and reserve
/// 64 KiB while every config file and inventory line read as satisfied. A knob
/// that cannot honour a value has to say so at BUILD time, where the person
/// who set it is standing — the alternative is a saving that looks applied and
/// is not, which is the defect class this campaign keeps finding.
///
/// Unset stays silent: the defaults are all above the floor.
fn env_usize_min(name: &str, default: usize, min: usize) -> usize {
    let v = env_usize(name, default);
    if v < min {
        panic!(
            "{name}={v} is below this pool's floor of {min}.\n  \
             The lookup path indexes the pool unconditionally, so a shorter one \
             is not representable — raise the value, or change the lookup to \
             refuse the entity kind first (issue 0827).\n  \
             This used to be silently rounded up to {min}, which reserved the \
             memory anyway while reading as though the knob had been honoured."
        );
    }
    v
}

fn env_usize(name: &str, default: usize) -> usize {
    match KCONFIG_KNOBS.iter().find(|(env, _)| *env == name) {
        Some((_, kconfig)) => nros_zephyr_build::knob_usize(name, kconfig, default),
        None => std::env::var(name)
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(default),
    }
}

/// phase-400 W6 — env, then the platform/board rung, then the builtin.
///
/// A thin sibling of `env_usize` rather than a change to it: the other callers
/// of that helper are knobs with no tenant, and giving them a rung parameter
/// they always pass `None` for would say the ladder reaches further than it
/// does.
fn env_usize_rung(name: &str, rung: Option<usize>, default: usize) -> usize {
    env_usize(name, rung.unwrap_or(default))
}

/// issue 1122 / 1199 — a `NROS_DECLARED_*` fact cmake derived for THIS image.
///
/// `None` when cmake made no claim, which is every build that is not driven by
/// our CMake lanes (a bare `cargo build`, a Rust leaf) and every configure
/// whose message-bound join REFUSED or answered on the `closure` basis. The
/// carrier is only written under `derived` + `subscribed`
/// (`_nros_payload_facts_env` in `cmake/NanoRosEntityFacts.cmake`), so an
/// absent variable is "no answer" and never "zero".
///
/// That distinction is the whole point: `0` is a legal and meaningful value
/// here -- it says this image's types all fit the small class -- so it cannot
/// share a spelling with "nobody told me".
fn declared_usize(name: &str) -> Option<usize> {
    std::env::var(name).ok().and_then(|v| v.trim().parse().ok())
}
