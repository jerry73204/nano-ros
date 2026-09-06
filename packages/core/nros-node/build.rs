//! Build script for nros-node
//!
//! Reads NROS_* environment variables and generates `nros_node_config.rs`
//! with compile-time configurable constants for executor and subscription sizing.
//!
//! Exports values via `links = "nros_node"` so dependents (nros-c, nros-cpp)
//! can read them as `DEP_NROS_NODE_*` environment variables.

use std::{env, path::Path};

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();

    println!("cargo:rustc-check-cfg=cfg(has_rmw)");
    // Emitted from the `needs-type-descriptors` capability feature (no dep
    // edge). Gates the descriptor-registration
    // schema-passing body + the `M: Message` super-bound for builds
    // where a descriptor-needing backend (Cyclone DDS) is linked. The
    // backend itself is brought into the link graph by the umbrella's
    // own `dep:nros-rmw-cyclonedds-sys`; the agnostic core only flips
    // this presence cfg.
    println!("cargo:rustc-check-cfg=cfg(rmw_needs_type_descriptors)");

    // The boot self-report (`src/boot_report.rs`) is OPT-IN, so an image that
    // does not ask for it is byte-identical to one built before it existed.
    // Set NROS_BOOT_REPORT=1 (Zephyr: CONFIG_NROS_BOOT_REPORT=y) to enable.
    println!("cargo:rustc-check-cfg=cfg(nros_boot_report)");
    println!("cargo:rerun-if-env-changed=NROS_BOOT_REPORT");
    if env::var("NROS_BOOT_REPORT").is_ok_and(|v| !v.is_empty() && v != "0" && v != "n") {
        println!("cargo:rustc-cfg=nros_boot_report");
    }

    // Emit `has_rmw` when an RMW seam is compiled in.
    //
    // phase-347 W1 — this used to test four features:
    // `CARGO_FEATURE_RMW_{ZENOH,XRCE,CFFI,UORB}`. **Three of them do not
    // exist.** This crate declares exactly one `rmw-*` selection feature,
    // `rmw-cffi`; there is no `rmw-zenoh`, `rmw-xrce` or `rmw-uorb` on
    // `nros-node`, so cargo could never set those env vars and the three
    // disjuncts were dead — vestiges of the pre-phase-248 shape, before the
    // umbrella converged on the cffi vtable.
    //
    // Deleting them is not a behaviour change; it is the removal of three
    // backend NAMES from a core package (RFC-0071: core receives capabilities,
    // it does not detect backends). `rmw-cffi` is already the capability —
    // "an RMW vtable seam is present" — wearing a four-backend disguise.
    //
    // The comment here also claimed `has_rmw` was set "when compiling for
    // tests (unit tests use MockSession)". No such branch existed, in this
    // revision or any other reachable from it; the claim was removed rather
    // than implemented, because unit tests that need a session already select
    // `rmw-cffi` through dev-dependencies.
    if env::var("CARGO_FEATURE_RMW_CFFI").is_ok() {
        println!("cargo:rustc-cfg=has_rmw");
    }

    // phase-347 W4 — signalled by the CAPABILITY feature
    // `needs-type-descriptors`, which the lowering enables when the selected
    // backend declares `type-descriptors` in its `nros-rmw.toml`. It was
    // `__cyclonedds-link`: a core feature named after one backend, flipping a
    // seam that was already capability-shaped. No `DEP_CYCLONEDDS_*` `links=`
    // probe anymore: the agnostic core has no Cargo dep on the Cyclone
    // crates, so there is no direct edge for cargo's `DEP_*` env-var
    // hand-off. The descriptor registration is a generic vtable seam
    // (`nros_rmw::register_type_descriptor`); the Cyclone backend
    // installs its registrar at init from its own crate.
    if env::var("CARGO_FEATURE_NEEDS_TYPE_DESCRIPTORS").is_ok() {
        println!("cargo:rustc-cfg=rmw_needs_type_descriptors");
    }

    // --- Primary user-facing knobs ---
    let max_cbs = env_usize("NROS_EXECUTOR_MAX_CBS", 4);
    let max_sc = env_usize("NROS_EXECUTOR_MAX_SC", 8);
    // Phase 214.C.3 — default coordinated with
    // `packages/rmw/zenoh/nros-rmw-zenoh/build.rs::ZPICO_SUBSCRIBER_BUFFER_SIZE`
    // (also 1024). If you change one, change the other — they share the
    // wire-format expectation. Both can be overridden independently via
    // their respective env vars.
    let rx_buf_size = env_usize("NROS_SUBSCRIPTION_BUFFER_SIZE", 1024);
    let param_svc_buf = env_usize("NROS_PARAM_SERVICE_BUFFER_SIZE", 4096);
    // Phase 104.C.2 — multi-Node-per-Executor (rclcpp `add_node`
    // pattern). Most apps run a single Node per Executor; bridge
    // nodes typically need 2 (ingress + egress). Default 4 leaves
    // headroom for multi-Node services with shared spin.
    let max_nodes = env_usize("NROS_EXECUTOR_MAX_NODES", 4);
    // issue 0790 — shutdown-hook slots, PER PHASE: the executor keeps one table
    // this size for pre-shutdown hooks and a second for on-shutdown hooks.
    //
    // Deliberately SMALL. Issue 0460 is the precedent: a per-entity static slot
    // is a measurable cost paid by every image, including the ones that never
    // register a single hook, so the default must not assume everyone wants
    // them. Two is enough for the canonical shape (park the actuator, release
    // the bus) and costs 2 x 2 x sizeof(fn ptr + ctx ptr) = 64 bytes on a
    // 64-bit host, 32 on a 32-bit target. Raise it with
    // `NROS_EXECUTOR_MAX_SHUTDOWN_CBS` (or `CONFIG_NROS_EXECUTOR_MAX_SHUTDOWN_CBS`
    // on Zephyr) when an image genuinely has more things to park.
    let max_shutdown_cbs = env_usize("NROS_EXECUTOR_MAX_SHUTDOWN_CBS", 2);
    // issue 0900 — how many of the MAX_CBS slots may hold an ACTION CLIENT,
    // the entity the arena derivation below budgets every slot at.
    //
    // Defaults to `max_cbs`, which reproduces the old `max_cbs * worst_case`
    // arithmetic byte for byte, so no existing image moves. It is a COUNT and
    // not a "which entity is heaviest" enum because Kconfig knobs are ints and
    // `knob_usize` is the one spelling that reaches the Zephyr Rust lane
    // (issue 0460); an enum would need a second reader shape for no gain.
    //
    // Setting it to 0 on a pub/sub-only image is the whole point: 74,240 bytes
    // becomes 16,384 at the defaults.
    //
    // phase-392 W6 — this comment used to end "and that arena is INLINE ON THE
    // TASK STACK, not in `.bss`". False since phase-271 (issue 0110), and it is
    // the reason the saving read as a stack saving rather than a RAM one. The
    // arena is a slice borrowed from caller-supplied backing; see
    // `executor::backing` for where that backing lives on each entry path.
    //
    // Too small fails at REGISTRATION (`NodeError::BufferTooSmall`), not at
    // link — same caveat `NROS_EXECUTOR_ARENA_SIZE` carries, and the reason
    // `Executor::arena_used()` plus the first-spin advisory landed first.
    let action_clients = env_usize("NROS_EXECUTOR_ACTION_CLIENTS", max_cbs).min(max_cbs);

    // --- Derived arena size ---
    // Arena must hold MAX_CBS entries. Worst-case entry is an
    // ActionClient: 3 CffiServiceClients (each carries a 4096-byte
    // `pending_request` blocking-fallback buffer + ~256 of header) +
    // 1 CffiSubscriber + 3 × rx_buf (goal/result/feedback) + ~256
    // entry overhead. Subscription / service entries are strictly
    // smaller, so budget every slot at the action-client size.
    // Per entry: 3 × (4096 + 384) + 3 × rx_buf + 1536 ≈ 14976 + 3·rx_buf
    //
    // Embedded targets that never instantiate an `ActionClient` can
    // override the derived size with `NROS_EXECUTOR_ARENA_SIZE`. A
    // pub/sub-only workload only needs `3 × rx_buf + 512` per entry.
    //
    // Phase 214.C.4 — magic-number breakdown for `4480` and friends:
    //   ACTION_CLIENT_SERVICE_BUF   = 4096  // pending_request blocking-fallback buf
    //   ACTION_CLIENT_HEADER_OVERHD =  384  // ~256 hdr + alignment slack
    //   ACTION_CLIENT_PER_SERVICE   = 4480  // = SERVICE_BUF + HEADER_OVERHD
    //   ACTION_CLIENT_SERVICES      =    3  // goal_send + cancel + get_result
    //   ACTION_CLIENT_SUB_OVERHEAD  = 1536  // 1 CffiSubscriber + ~256 entry slop
    const ACTION_CLIENT_PER_SERVICE: usize = 4096 + 384;
    const ACTION_CLIENT_SERVICES: usize = 3;
    const ACTION_CLIENT_FEEDBACK_SUBS: usize = 3; // goal + result + feedback rx
    const ACTION_CLIENT_SUB_OVERHEAD: usize = 1536;
    const ARENA_BASE_OVERHEAD: usize = 2048;
    const ARENA_FLOOR: usize = 8192;
    // issue 0900 — the budget for the two entry SHAPES, summed over how many
    // slots each may occupy, instead of charging every slot the larger one.
    // With `action_clients == max_cbs` (the default) the second term is zero
    // and this is byte-identical to the old formula.
    const PUBSUB_SUB_BUFS: usize = 3;
    const PUBSUB_ENTRY_OVERHEAD: usize = 512;
    let action_client_entry = ACTION_CLIENT_SERVICES * ACTION_CLIENT_PER_SERVICE
        + ACTION_CLIENT_FEEDBACK_SUBS * rx_buf_size
        + ACTION_CLIENT_SUB_OVERHEAD;
    // phase-403 step 3 -- a SUBSCRIPTION's arena buffer is sized by what it
    // RECEIVES, which is not the knob this term used.
    //
    // `NROS_SUBSCRIPTION_BUFFER_SIZE` is also `DEFAULT_TX_BUF` and every raw
    // entity's default buffer, so it is derived over the whole LINKED closure
    // -- a type this image only publishes still has to fit. The arena's
    // receive region is not that: `register_subscription_buffered_*` sizes it
    // from the type's own bound, and the SUBSCRIBER payload class is the
    // maximum over subscribed types, so it is a tight upper bound for every
    // subscription and the closure figure is not.
    //
    // On the reference island that is 880 against 1496, and the difference is
    // 3 * 616 per subscription -- 18,480 bytes over ten of them, which is why
    // that image's hand-set 40,960 sits BELOW the modelled 52,304.
    //
    // Falls back to the closure knob when the payload class is absent, which
    // is larger and therefore the safe direction.
    // NOT WIRED YET, and the fallback is deliberate. The receive class is
    // resolved into the cargo env as `ZPICO_SUBSCRIBER_BUFFER_SIZE` -- a
    // BACKEND name, which this backend-agnostic crate must not read -- so
    // `NROS_SUBSCRIBER_BUFFER_SIZE` is absent here and this falls back to the
    // closure knob. Measured on the reference island: the arena derives 52,304
    // against a hand-set 40,960, and the whole 11,344-byte difference is this
    // term using 1,496 where the receive class is 880.
    //
    // Giving it a backend-agnostic spelling is a DESIGN choice with two bad
    // options -- have this crate read a ZPICO_ name, or resolve one value
    // under two names and invite the drift that three separate double
    // resolutions have already caused in this file's sibling. It is named in
    // phase-403 step 3 rather than guessed at here.
    let rx_recv_size = env_usize("NROS_SUBSCRIBER_BUFFER_SIZE", rx_buf_size);
    let pubsub_entry = PUBSUB_SUB_BUFS * rx_recv_size + PUBSUB_ENTRY_OVERHEAD;

    // phase-403 step 3 -- SUM OVER WHAT THE IMAGE DECLARES, when it declares.
    //
    // The arithmetic below this comment charges EVERY callback slot at the
    // pub/sub entry size. That is a worst case over `max_cbs`, and on an image
    // that declares its entities it is wrong in a measurable direction: a
    // TIMER claims 32 bytes of arena (measured, `report_arena_costs`) and this
    // model bills it `3 * rx_buf + 512` -- 5,000 bytes at the reference
    // island's derived rx_buf of 1,496. Ten subscriptions and four timers cost
    // 72,048 modelled against ~50,000 actually claimed, which is why that
    // image pins the knob by hand.
    //
    // So when phase-403 W9's entity inventory reaches this lane, the sum runs
    // per KIND instead. Absence is not zero: a count that is missing means
    // "nobody declared", and the whole per-kind path is skipped rather than
    // summing zeros into a too-small arena.
    //
    // TIMER_ENTRY is measured, not modelled. `report_arena_costs` reports 32
    // on x86_64; entry structs hold pointers, so this is rounded UP to a
    // pointer-generous 64 rather than pinned to the host figure -- the one
    // direction that cannot under-size a 32-bit target.
    const TIMER_ENTRY: usize = 64;
    let declared_subs = env_opt_usize("NROS_ENTITY_COUNT_SUBSCRIPTION");
    let declared_timers = env_opt_usize("NROS_ENTITY_COUNT_TIMER");
    let declared_services = env_opt_usize("NROS_ENTITY_COUNT_SERVICE_SERVER");
    let declared_action_clients = env_opt_usize("NROS_ENTITY_COUNT_ACTION_CLIENT");
    let declared_action_servers = env_opt_usize("NROS_ENTITY_COUNT_ACTION_SERVER");

    let derived_arena = match (
        declared_subs,
        declared_timers,
        declared_services,
        declared_action_clients,
        declared_action_servers,
    ) {
        (Some(subs), Some(timers), Some(services), Some(acl), Some(asv)) => {
            // A service server carries a request AND a reply buffer, so it is
            // billed at the pub/sub entry plus one more buffer rather than
            // being folded into the pub/sub term.
            let service_entry = pubsub_entry + rx_buf_size;
            (subs * pubsub_entry
                + timers * TIMER_ENTRY
                + services * service_entry
                + (acl + asv) * action_client_entry
                + ARENA_BASE_OVERHEAD)
                .max(ARENA_FLOOR)
        }
        // Nobody declared, or declared only partly: keep the pre-step-3
        // arithmetic byte for byte, so no existing image moves.
        _ => (action_clients * action_client_entry
            + max_cbs.saturating_sub(action_clients) * pubsub_entry
            + ARENA_BASE_OVERHEAD)
            .max(ARENA_FLOOR),
    };
    // `0` is the Kconfig SENTINEL for "derive it" (zephyr/Kconfig:
    // NROS_EXECUTOR_ARENA_SIZE, "0 = derive"), and it has to be honoured HERE,
    // where the value is consumed.
    //
    // `nros_cargo_build.cmake` already knows the sentinel and deliberately does
    // not forward a literal 0 — "forwarding a literal 0 would hand it a
    // zero-byte arena rather than the derivation". That guard became INERT when
    // issue 0460 made `knob_usize` read `$DOTCONFIG` directly so knobs could
    // reach the Rust lane at all: build.rs now finds `CONFIG_..._ARENA_SIZE=0`
    // in `.config` whether or not cmake exported it, and took it literally.
    //
    // The result was a zero-byte arena on every Zephyr image built with the
    // default: the FIRST node registers, the second fails
    // `NodeError::BufferTooSmall`, and the entry panics. Kconfig's own help
    // predicted the shape — "too small fails at runtime, not at link".
    let arena_size = match env_usize("NROS_EXECUTOR_ARENA_SIZE", derived_arena) {
        0 => derived_arena,
        n => n,
    };

    let contents = format!(
        "/// Maximum number of executor callback slots \
         (set via NROS_EXECUTOR_MAX_CBS, default 4).\n\
         pub const MAX_CBS: usize = {max_cbs};\n\
         \n\
         /// Maximum number of `SchedContext` slots per executor \
         (set via NROS_EXECUTOR_MAX_SC, default 8). Phase 110.B.\n\
         pub const MAX_SC: usize = {max_sc};\n\
         \n\
         /// Executor arena size in bytes (derived from MAX_CBS, RX_BUF_SIZE \
         and ACTION_CLIENTS).\n\
         pub const ARENA_SIZE: usize = {arena_size};\n\
         \n\
         /// How many callback slots the arena derivation budgeted at \
         ActionClient size (set via NROS_EXECUTOR_ACTION_CLIENTS, default \
         MAX_CBS). Issue 0900.\n\
         pub const ARENA_ACTION_CLIENTS: usize = {action_clients};\n\
         \n\
         /// Default subscription receive buffer size in bytes \
         (set via NROS_SUBSCRIPTION_BUFFER_SIZE, default 1024).\n\
         pub const DEFAULT_RX_BUF_SIZE: usize = {rx_buf_size};\n\
         \n\
         /// Parameter service request/reply buffer size in bytes \
         (set via NROS_PARAM_SERVICE_BUFFER_SIZE, default 4096).\n\
         pub const PARAM_SERVICE_BUFFER_SIZE: usize = {param_svc_buf};\n\
         \n\
         /// Maximum number of Nodes attached to a single Executor \
         (set via NROS_EXECUTOR_MAX_NODES, default 4). Phase 104.C.2.\n\
         pub const MAX_NODES: usize = {max_nodes};\n\
         \n\
         /// Shutdown-hook slots PER PHASE -- one table this size for \
         pre-shutdown hooks and a second for on-shutdown hooks \
         (set via NROS_EXECUTOR_MAX_SHUTDOWN_CBS, default 2). Issue 0790.\n\
         pub const MAX_SHUTDOWN_CBS: usize = {max_shutdown_cbs};\n"
    );

    std::fs::write(Path::new(&out_dir).join("nros_node_config.rs"), contents).unwrap();

    emit_executor_backing(&out_dir);

    // Export via `links = "nros_node"` so dependents (nros-c, nros-cpp)
    // can read these as DEP_NROS_NODE_MAX_CBS, DEP_NROS_NODE_ARENA_SIZE, etc.
    println!("cargo:max_cbs={max_cbs}");
    println!("cargo:arena_size={arena_size}");
    println!("cargo:rx_buf_size={rx_buf_size}");
}

/// phase-392 W6 — emit the ONE item that has to be generated for
/// `executor::backing`: the named static itself, because
/// `#[unsafe(link_section = …)]` takes a string LITERAL and the section name is
/// a build-time input.
///
/// `NROS_EXECUTOR_BACKING_SECTION` is a SECTION NAME, not a path, so watching
/// the env value is the correct fingerprint here (issue 0491 is about paths,
/// which have several spellings for one directory; a section name has one).
///
/// The generated file is deliberately tiny and carries no arithmetic — the size
/// is `EXECUTOR_BACKING_U64S`, a `const` in the module, so a reader looking for
/// "how big is it?" never has to open `OUT_DIR`.
fn emit_executor_backing(out_dir: &str) {
    // Words to reserve. ABSENT means "the crate's own default sizing", which is
    // why this is `env_opt_usize` and not `env_usize` with a number: build.rs
    // cannot call `ExecutorSizing::DEFAULT.u64_len()` (it lives in the crate it
    // is building), and RETYPING that arithmetic here is the single-derivation
    // rule's exact failure mode — a second copy that agrees until a table moves.
    // So absence emits the const EXPRESSION and the crate computes it.
    //
    // **ZERO means "no static; leak from the heap as before"**, and it is the
    // documented opt-out for a memory-constrained image: on a hosted target the
    // allocator has no fixed reservation, so the static is free, but on an RTOS
    // the allocator arena IS a fixed static (`CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE`
    // on Zephyr, `configTOTAL_HEAP_SIZE` on FreeRTOS) and it is sized to hold
    // this backing already. Turning the static on there without lowering that
    // knob reserves the same bytes twice. The pairing is per-image and nobody
    // but the image's author can measure it — see phase-392 W6.
    //
    // Issue 1171 -- `zephyr/Kconfig` now declares
    // `CONFIG_NROS_EXECUTOR_BACKING_U64S`, so a Zephyr image can STATE the
    // reservation and lower `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` by exactly
    // `8 * words`, which is a pairing that cannot drift when an executor knob
    // moves and is the same on every board. `-1` is the tree's DERIVE sentinel
    // and reaches this as `None` (it does not parse as a `usize`, which is how
    // every `-1 = derive` knob here falls through), so the shipped Kconfig
    // default is still "the crate's own sizing".
    let words = env_opt_usize("NROS_EXECUTOR_BACKING_U64S");
    println!("cargo:rustc-check-cfg=cfg(nros_executor_backing_static)");
    if words == Some(0) {
        // No item at all, and no cfg: `backing::take` is then a `None` stub and
        // the crate carries no reservation. The file still has to EXIST because
        // `include!` is unconditional at the module's top level.
        std::fs::write(
            Path::new(out_dir).join("nros_executor_backing.rs"),
            "// NROS_EXECUTOR_BACKING_U64S=0 -- no static reservation; the alloc\n\
             // convenience constructors leak from the heap (phase-392 W6).\n",
        )
        .unwrap();
        return;
    }
    println!("cargo:rustc-cfg=nros_executor_backing_static");
    let len = match words {
        Some(n) => n.to_string(),
        None => "EXECUTOR_BACKING_DEFAULT_U64S".to_string(),
    };
    println!("cargo:rerun-if-env-changed=NROS_EXECUTOR_BACKING_SECTION");
    let section = env::var("NROS_EXECUTOR_BACKING_SECTION").unwrap_or_default();
    let attr = if section.is_empty() {
        String::new()
    } else {
        // A section name reaches the assembler verbatim, so refuse anything that
        // could close the string literal or smuggle a directive: a mangled
        // section is a link error four layers from here, or worse, a silently
        // misplaced 74 KiB.
        assert!(
            section
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '$')),
            "NROS_EXECUTOR_BACKING_SECTION must be [A-Za-z0-9._$]+, got {section:?}"
        );
        format!("#[unsafe(link_section = \"{section}\")]\n")
    };
    let contents = format!(
        "/// GENERATED by `nros-node/build.rs` -- see `executor::backing` for what \
         this is\n\
         /// and why it is generated rather than written.\n\
         ///\n\
         /// The executor's per-entry storage, reserved ONCE in `.bss` so \
         `mem-report`\n\
         /// can price what a leaked `Box` made invisible (phase-392 W6, RFC-0002 \
         SS 4.4b).\n\
         pub(crate) const EXECUTOR_BACKING_U64S: usize = {len};\n\
         {attr}\
         static mut EXECUTOR_BACKING: [MaybeUninit<u64>; EXECUTOR_BACKING_U64S] =\n    \
         [MaybeUninit::uninit(); EXECUTOR_BACKING_U64S];\n"
    );
    std::fs::write(
        Path::new(out_dir).join("nros_executor_backing.rs"),
        contents,
    )
    .unwrap();
}

/// The platform and board rungs of the RFC-0049 ladder for `[knobs.executor]`.
///
/// phase-400 W6. This crate deliberately has NO `platform-*` cargo feature
/// (phase-248 C2: the core executor is platform-agnostic and reaches the
/// platform through the vtable), so the build script cannot know its platform
/// from a `cfg`. `BuildRungs` learns it the way every other build-script fact
/// travels here — the lane exports a value and a POINTER, and the script reads
/// the file — and it is SHARED: this function grew its own copy of that dance
/// first, and two build scripts resolving the same rungs differently is the
/// drift `check-knob-single-reader` exists to catch one level up.
///
/// Absent pointer (a bare `cargo build` with no lane) → every rung is `None`
/// and the front-end below decides, exactly as before.
fn executor_rungs() -> nros_board_common::platform_config::ExecutorKnobs {
    nros_board_common::platform_config::BuildRungs::from_build_env()
        .map(|r| r.executor_rungs())
        .unwrap_or_default()
}

/// The knob a front-end env name belongs to, derived from the one table that
/// maps the other way rather than retyped.
fn knob_for_env(name: &str) -> Option<&'static str> {
    nros_board_common::platform_config::EXECUTOR_KNOBS
        .iter()
        .copied()
        .find(|k| nros_board_common::platform_config::executor_env_key(k) == name)
}

fn rung_value(
    rungs: &nros_board_common::platform_config::ExecutorKnobs,
    knob: &str,
) -> Option<usize> {
    match knob {
        "max_cbs" => rungs.max_cbs,
        "max_sc" => rungs.max_sc,
        "max_nodes" => rungs.max_nodes,
        "max_shutdown_cbs" => rungs.max_shutdown_cbs,
        "action_clients" => rungs.action_clients,
        "arena_size" => rungs.arena_size,
        "subscription_buffer_size" => rungs.subscription_buffer_size,
        "param_service_buffer_size" => rungs.param_service_buffer_size,
        _ => None,
    }
}

/// A DECLARED entity count, or `None` when nobody declared one.
///
/// phase-403 step 3. Deliberately NOT `env_usize` with a default: absence and
/// zero are different answers here. `Some(0)` means "this image declares no
/// timers"; `None` means "no entity inventory reached this lane", and summing
/// a missing count as zero would produce an arena too small for entities that
/// exist. The caller therefore requires EVERY count before using the per-kind
/// sum, and otherwise keeps the pre-step-3 worst case.
///
/// Reads the same two places `env_usize` does and in the same order, so an
/// operator override still wins: the cargo env, then `$DOTCONFIG` (issue 0460
/// -- knobs reach the Zephyr Rust lane only through the dotconfig, because
/// `set(ENV{...})` at configure time does not survive into the cargo build).
fn env_opt_usize(name: &str) -> Option<usize> {
    println!("cargo:rerun-if-env-changed={name}");
    if let Some(v) = std::env::var(name).ok().and_then(|v| v.trim().parse().ok()) {
        return Some(v);
    }
    nros_zephyr_build::dotconfig_usize(&format!("CONFIG_{name}"))
}

/// One executor knob: env → Kconfig → board → platform → built-in default.
///
/// The front-end keeps winning. Migrating a knob into the ladder must not take
/// an operator's override away, which is half of this wave's own gate.
fn env_usize(name: &str, default: usize) -> usize {
    println!("cargo:rerun-if-env-changed={name}");
    if let Some(v) = std::env::var(name).ok().and_then(|v| v.trim().parse().ok()) {
        return v;
    }
    if let Some(v) = nros_zephyr_build::dotconfig_usize(&format!("CONFIG_{name}")) {
        return v;
    }
    static RUNGS: std::sync::OnceLock<nros_board_common::platform_config::ExecutorKnobs> =
        std::sync::OnceLock::new();
    let rungs = RUNGS.get_or_init(executor_rungs);
    knob_for_env(name)
        .and_then(|k| rung_value(rungs, k))
        .unwrap_or(default)
}
