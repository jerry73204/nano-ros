//! RFC-0051 / phase-295 W1 — THE test matrix (single source of truth).
//!
//! Every runtime e2e lane in `nros-tests` is a **cell** of this table:
//! (platform × language × RMW × workload × kind). The parametrized matrix
//! consumers (`example_e2e`, `workspace_e2e`, …) iterate [`CELLS`]; the
//! isolation allocator ([`crate::alloc`]) derives each cell's port/domain;
//! the coverage gate cross-checks `examples/fixtures.toml` against this
//! table in BOTH directions. A gap in coverage is a visible
//! [`Tier::BuildOnly`] / [`Tier::CarveOut`] row here — never an absent
//! file (the pre-295 failure mode: nobody can see a test that doesn't
//! exist).
//!
//! Rules:
//! - Carve-outs carry their REASON in the table (audit E5: no
//!   tribal-memory carve-outs).
//! - New platform / language / RMW support adds cells HERE first; the
//!   matrix consumer then runs them without new test files (audit E6).
//! - `Workload` values map 1:1 onto the stock-ROS-demo behavior contracts
//!   the shared checker asserts (audit E7).

use crate::platform::{TestLang, TestVariant};

/// Platform axis. Extends the historical `platform.rs` QEMU set with the
/// native / emulator / hardware targets so the WHOLE lane inventory lives
/// in one axis.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PlatformId {
    /// The Linux host board over the `posix` platform. Isolation is EPHEMERAL
    /// (ports/domains picked at runtime) — the allocator's baked formula does
    /// not apply.
    ///
    /// phase-337 W8.b renamed this from `Native` (RFC-0064 R3): "native"
    /// implies any general-purpose OS, and Windows/macOS do not build. The
    /// PLATFORM is still `posix` — only the board-level promise is narrowed to
    /// what `just ci` actually exercises.
    Linux,
    /// Zephyr native_sim (NSOS host sockets).
    ZephyrNativeSim,
    /// Zephyr on QEMU MPS2-AN385 (Cortex-M3) — Zephyr's OWN IP stack over the
    /// `eth_smsc911x` driver, 32-bit.
    ///
    /// phase-337 W2 added this because "Zephyr" previously meant exactly one
    /// config: `native_sim/native/64`, where sockets are OFFLOADED to the host
    /// and the pointer width is the host's. That is a board, not a platform —
    /// and the difference is not academic. Bringing this witness up cost five
    /// real defects (a 32-bit `size_t`/`uintptr_t` header conflict, an atomics
    /// feature gated on an arch list, a staticlib with no allocator or panic
    /// handler off native_sim, a duplicated cmake feature string, and a board
    /// with no entropy device), every one of them invisible to native_sim.
    ///
    /// Cells here are C/C++ only: the pinned `zephyr-lang-rust` cannot compile
    /// for any board whose devicetree has gpio nodes (issue 0432).
    ZephyrQemuCortexM,
    /// FreeRTOS on QEMU MPS2-AN385 (lwIP).
    FreertosMps2,
    /// FreeRTOS POSIX simulator on the build host (phase-370).
    ///
    /// The kernel's `ThirdParty/GCC/Posix` port: FreeRTOS tasks are pthreads,
    /// the tick is a host timer signal, and the network stack is the HOST's
    /// rather than lwIP. That last one is why it is a separate witness from
    /// `FreertosMps2` and not a config of it — the RMW below it is the `posix`
    /// branch's host ddsrt, so the two exercise different halves of the family.
    ///
    /// Cells here are C/C++ only: the POSIX port has no Rust entry shape of its
    /// own, and this board carries no Rust board crate.
    FreertosPosix,
    /// NuttX on QEMU arm virt (Cortex-A7).
    NuttxArm,
    /// NuttX on QEMU rv-virt (riscv32).
    NuttxRiscv,
    /// ThreadX Linux simulation (host sockets).
    ThreadxLinux,
    /// ThreadX on QEMU riscv64 virt (NetX Duo).
    ThreadxRiscv64,
    /// ESP32-C3 under the Espressif QEMU fork (open_eth).
    Esp32Qemu,
    /// Bare-metal RTIC on QEMU MPS2-AN385.
    QemuBaremetal,
    /// ARM FVP Base_RevC AEMv8-R (license-gated model).
    Fvp,
    /// PX4-SITL host (the uORB middleware). Issue 0341 — expressible so the
    /// uORB axis has a home; carried as a CarveOut (no CI runner builds SITL).
    Px4,
}

impl PlatformId {
    /// Stable index for the allocator formulas. Bounded — extending the
    /// enum extends the port/domain bands; the injectivity gate re-proves
    /// collision-freedom on every run.
    pub const fn index(self) -> u16 {
        match self {
            PlatformId::Linux => 0,
            PlatformId::ZephyrNativeSim => 1,
            PlatformId::FreertosMps2 => 2,
            PlatformId::NuttxArm => 3,
            PlatformId::NuttxRiscv => 4,
            PlatformId::ThreadxLinux => 5,
            PlatformId::ThreadxRiscv64 => 6,
            PlatformId::Esp32Qemu => 7,
            PlatformId::QemuBaremetal => 8,
            // 9 was `Stm32F4` until phase-337 W7.a. Renumbering `Fvp`/`Px4`
            // down into the gap is free: both carry ZERO Runtime cells, so no
            // fixture image, locator or domain was ever baked from their band.
            //
            // phase-337 W2.c takes 9 for the same reason, and moves them again.
            // The index is NOT free real estate: `alloc::domain_of` gives every
            // platform a 21-wide window out of the 232 DDS domains, which fits
            // exactly 11 — so a twelfth platform at index 11 computes domain 233
            // and `domains_valid` rejects it. The scarce resource is therefore a
            // LOW index, and it belongs to platforms that actually bake. `Fvp`
            // and `Px4` still bake nothing (tier 3 / CarveOut), so their windows
            // remain unreachable arithmetic wherever they sit; a witness with
            // Runtime cells cannot say the same. If either ever gains a Runtime
            // cell, `domains_valid` fails immediately — which is the right
            // answer, because at that point the window scheme is genuinely full
            // and needs narrowing, not another renumber.
            PlatformId::ZephyrQemuCortexM => 9,
            // phase-370 takes 10 on the rule the paragraph above states: a LOW
            // index is the scarce resource and belongs to a platform that
            // BAKES. `domain_of` is `1 + index*21 + slot*3 + lang`, so index 10
            // tops out at 231 and index 11 at 252 — 10 is the last usable slot,
            // and it was held by a tier-3 witness with zero Runtime cells.
            // `Fvp` and `Px4` move up into arithmetic that is unreachable for
            // exactly as long as they bake nothing, which is the same trade the
            // comment above already made on their behalf.
            PlatformId::FreertosPosix => 10,
            PlatformId::Fvp => 11,
            PlatformId::Px4 => 12,
        }
    }

    /// The `platform = "..."` token(s) `examples/fixtures.toml` spells this
    /// platform with — the SSoT for that vocabulary, in both directions
    /// ([`PlatformId::from_fixture_token`] is its inverse, gated by
    /// `fixture_token_mapping_round_trips`).
    ///
    /// It is one-to-MANY: `Esp32Qemu` covers both the RTOS lane (`esp32`) and the
    /// bare-metal one (`qemu-esp32-baremetal`). Selecting the platform must select
    /// both, so callers iterate the slice rather than taking a single token.
    ///
    /// One home on purpose. Before phase-318 W4.d this existed only as
    /// `platform_from_str` inside `tests/matrix_fixture_coverage.rs`, and the
    /// forward direction got hand-written a second time — with
    /// `qemu-esp32-baremetal` attributed to the wrong platform. A second spelling
    /// of a mapping is the recurring defect class in this repo (CLAUDE.md "add ONE
    /// shared helper rather than a second spelling").
    pub const fn fixture_tokens(self) -> &'static [&'static str] {
        match self {
            // phase-337 W8.c — moved from `native` WITH the two other
            // token-derived vocabularies, because the token is not only a
            // token: it is also the first argument of
            // `scripts/build/fixtures-build.sh` / `workspace-fixtures-build.sh`,
            // the scope name in `fixture-make-driver.sh`, and the coordinate
            // prefix `fixture-lane.sh` greps. Renaming any one of those alone
            // would leave rows saying `linux` and the builder that produces them
            // still called with `native` — two spellings of one fact.
            //
            // What did NOT move, and is not an oversight: the `native` LANE
            // (`lane=native`, `_NROS_LANES`), the `just` MODULE (`just native
            // …`, see `just_module` below), and the `examples/native/`
            // DIRECTORY — that last one is a `dir =` value, not this token.
            // Those are different vocabularies that merely share a spelling.
            PlatformId::Linux => &["linux"],
            PlatformId::ZephyrNativeSim => &["zephyr"],
            // phase-337 W2.c/W2.e declared this token with NO `fixtures.toml`
            // row spelling it, because the board was built by the west leaves
            // lane off its own bash matrix. phase-350 W1 gave that lane rows, so
            // three `builder = "west"` rows spell it now (the mps2 talker
            // witness, one per language).
            //
            // Those rows are why the token is not just cosmetic: the witness
            // leaves spell the SAME `dir`, `lang` and `rmw` as their native_sim
            // siblings, so putting both boards under `zephyr` would give two
            // distinct rows one coordinate — and a coordinate is what every
            // lane-scoped build and the whole staleness gate select on.
            PlatformId::ZephyrQemuCortexM => &["zephyr-cortex-m"],
            PlatformId::FreertosMps2 => &["freertos"],
            // A token of its own, for the reason the ZephyrQemuCortexM comment
            // above gives: the witness leaves spell the same `lang` and `rmw`
            // as their mps2 siblings, so one shared token would give two
            // distinct rows one coordinate — and a coordinate is what every
            // lane-scoped build and the staleness gate select on.
            PlatformId::FreertosPosix => &["freertos-posix"],
            PlatformId::NuttxArm => &["nuttx"],
            PlatformId::NuttxRiscv => &["nuttx-riscv"],
            PlatformId::ThreadxLinux => &["threadx-linux"],
            PlatformId::ThreadxRiscv64 => &["threadx-riscv64"],
            PlatformId::Esp32Qemu => &["esp32", "qemu-esp32-baremetal"],
            PlatformId::QemuBaremetal => &["qemu-arm-baremetal"],
            PlatformId::Fvp => &["fvp"],
            // Carried as a CarveOut (no CI runner builds PX4-SITL), so no row
            // spells this today. The token is still declared: the vocabulary has
            // to name every platform, or a lane that later gains PX4 fixtures
            // selects nothing for them and looks fast rather than broken.
            PlatformId::Px4 => &["px4"],
        }
    }

    /// The `just` module that owns this platform's build/test verbs — the one a
    /// CI job runs as `just <module> …`.
    ///
    /// A THIRD vocabulary after `PlatformId` and the fixtures.toml tokens, so it
    /// lives here with the other two rather than being hand-listed in a workflow
    /// yml, where nothing would notice it going stale. `nightly.yml`'s platform
    /// list was hand-written exactly that way.
    ///
    /// Not injective: `NuttxArm` and `NuttxRiscv` share `nuttx` (which owns
    /// `build-riscv-*`), and `Fvp` is built by `just zephyr build-fvp-*`. Callers
    /// that need a job list must dedupe.
    pub const fn just_module(self) -> &'static str {
        match self {
            // The `just` MODULE keeps the name `native`: it is a third
            // vocabulary naming a LANE, ~100 CI references deep, and RFC-0064's
            // decision is about the board's promise, not the recipe namespace.
            PlatformId::Linux => "native",
            // All three are `just zephyr …` — one module, three boards. That is
            // the RFC-0064 shape working as intended: a board is a conf bundle
            // under a family's recipes, not a namespace of its own.
            PlatformId::ZephyrNativeSim | PlatformId::Fvp | PlatformId::ZephyrQemuCortexM => {
                "zephyr"
            }
            // One module, two boards — the RFC-0064 shape, same as the three
            // Zephyr witnesses above. `just freertos build-posix-*` owns it.
            PlatformId::FreertosMps2 | PlatformId::FreertosPosix => "freertos",
            PlatformId::NuttxArm | PlatformId::NuttxRiscv => "nuttx",
            PlatformId::ThreadxLinux => "threadx_linux",
            PlatformId::ThreadxRiscv64 => "threadx_riscv64",
            PlatformId::Esp32Qemu => "esp32",
            PlatformId::QemuBaremetal => "qemu",
            PlatformId::Px4 => "px4",
        }
    }

    /// `examples/fixtures.toml` `platform` string → matrix platform. Inverse of
    /// [`PlatformId::fixture_tokens`].
    pub fn from_fixture_token(s: &str) -> Option<PlatformId> {
        PlatformId::ALL
            .iter()
            .copied()
            .find(|p| p.fixture_tokens().contains(&s))
    }

    pub const ALL: &'static [PlatformId] = &[
        PlatformId::Linux,
        PlatformId::ZephyrNativeSim,
        PlatformId::ZephyrQemuCortexM,
        PlatformId::FreertosMps2,
        PlatformId::FreertosPosix,
        PlatformId::NuttxArm,
        PlatformId::NuttxRiscv,
        PlatformId::ThreadxLinux,
        PlatformId::ThreadxRiscv64,
        PlatformId::Esp32Qemu,
        PlatformId::QemuBaremetal,
        PlatformId::Fvp,
        PlatformId::Px4,
    ];
}

/// RMW axis.
///
/// Issue 0341 — `Uorb` is declared supported in ARCHITECTURE §2
/// (`rmw-{zenoh,xrce,cyclonedds,uorb}`) with a real crate
/// (`packages/rmw/uorb/nros-rmw-uorb`) and example
/// (`packages/testing/nros-px4-register-check`), so it
/// must be *expressible* in the matrix. It carries a documented CarveOut cell
/// rather than a Runtime lane: uORB runs inside a PX4-SITL build that no CI
/// runner here provides. An expressible-but-carved-out axis is honest; an
/// inexpressible one hides the gap.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Rmw {
    Zenoh,
    Cyclonedds,
    Xrce,
    Uorb,
}

impl Rmw {
    pub const fn index(self) -> u16 {
        match self {
            Rmw::Zenoh => 0,
            Rmw::Cyclonedds => 1,
            Rmw::Xrce => 2,
            Rmw::Uorb => 3,
        }
    }
}

/// Language axis. `Mixed` exists only for `Kind::Workspace` cells.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Lang {
    Rust,
    C,
    Cpp,
    Mixed,
}

impl Lang {
    /// Maps onto the historical [`TestLang`] port multiplier, extended
    /// with a fourth column for `Mixed` (the injectivity gate caught the
    /// original share-the-rust-slot idea colliding on platforms that run
    /// BOTH a rust and a mixed workspace cell — e.g. zephyr EntryPubsub).
    pub const fn port_index(self) -> u16 {
        match self {
            Lang::Rust => 0,
            Lang::C => 1,
            Lang::Cpp => 2,
            Lang::Mixed => 3,
        }
    }

    pub const fn as_test_lang(self) -> TestLang {
        match self {
            Lang::Rust | Lang::Mixed => TestLang::Rust,
            Lang::C => TestLang::C,
            Lang::Cpp => TestLang::Cpp,
        }
    }

    /// The axis token — the spelling used in fixture paths, cell labels and
    /// failure messages.
    ///
    /// phase-373 W3: eight tests carried a byte-identical private `lang_str`
    /// for this. A pure mapping over an enum this crate owns belongs on the
    /// enum, where a new variant makes the compiler ask about it once rather
    /// than leaving eight copies to be found by hand.
    ///
    /// Note this is deliberately NOT `Display`: the sibling `Rmw` mapping is
    /// genuinely ambiguous (`Cyclonedds` is spelled `"cyclone"` by the native
    /// example consumers and `"cyclonedds"` by `zephyr.rs`), and giving one
    /// axis a blessed `Display` while its neighbour keeps per-consumer
    /// spellings would imply an agreement that does not exist.
    pub const fn as_str(self) -> &'static str {
        match self {
            Lang::Rust => "rust",
            Lang::C => "c",
            Lang::Cpp => "cpp",
            Lang::Mixed => "mixed",
        }
    }
}

/// Workload axis — each value is a stock-ROS-demo behavior contract the
/// shared checker knows how to assert (RFC-0051 §2).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Workload {
    Pubsub,
    Service,
    Action,
    /// Workspace Entry boot + pubsub delivery (the `zephyr_entry` class).
    EntryPubsub,
    CustomMsg,
    Logging,
    Qos,
    Params,
    Lifecycle,
    Safety,
    RealtimeTiers,
    Multihost,
    /// Launch/model `<remap>` + `~` private names reach the WIRE remapped
    /// (phase-306 W4, issue 0255).
    Remap,
    /// Platform-ABI capability, no messaging: two tasks spawned through
    /// `nros_platform_task_init` prove `errno` is PER-THREAD (issue 0680).
    /// Single image, self-contained, no peer.
    Errno,
    /// phase-381 — READ the ROS graph. The node enumerates a live stock
    /// `rmw_zenoh_cpp` peer and is compared against `ros2 node list`.
    ///
    /// Its own workload because it is the only one whose subject is DISCOVERY
    /// rather than delivery: every other cell asks "did the message arrive",
    /// this one asks "can we see who is there", and the two fail for different
    /// reasons.
    Graph,
    /// phase-433 W6 — what a node ADVERTISES about itself, cross-checked
    /// against what a live ROS 2 peer reads for the same entities: matched
    /// counts, the publisher GID, the actual-QoS read-back (issue 0823) and
    /// `get_serialization_format`.
    ///
    /// Its own workload for the same reason [`Workload::Graph`] is: the
    /// subject is neither delivery nor discovery but SELF-REPORT. Every slot
    /// in it is one nobody can check from inside one process — a matched count
    /// is a statement about a peer, a GID's whole purpose is that another
    /// participant recognises it, and a granted QoS is only interesting where
    /// it differs from the requested one.
    AdvertisedState,
}

impl Workload {
    /// Port-band offset. Pubsub/Service/Action keep the historical
    /// variant offsets (0/10/20); the workspace workloads take the
    /// 30..=110 band within each platform's lang column (stride 100 —
    /// bands never overlap the variant offsets).
    pub const fn port_offset(self) -> u16 {
        match self {
            Workload::Pubsub => 0,
            Workload::Service => 10,
            Workload::Action => 20,
            Workload::EntryPubsub => 30,
            Workload::CustomMsg => 40,
            Workload::Logging => 50,
            Workload::Qos => 60,
            Workload::Params => 70,
            Workload::Lifecycle => 80,
            Workload::Safety => 90,
            Workload::RealtimeTiers => 91,
            Workload::Multihost => 92,
            Workload::Remap => 93,
            // Needs no port: the fixture opens no socket and has no peer.
            // The offset only has to be unique within the band.
            Workload::Errno => 94,
            Workload::Graph => 95,
            Workload::AdvertisedState => 96,
        }
    }

    /// Maps the three classic variants onto the historical enum (the
    /// QEMU harness APIs still take [`TestVariant`]).
    pub const fn as_test_variant(self) -> Option<TestVariant> {
        match self {
            Workload::Pubsub => Some(TestVariant::Pubsub),
            Workload::Service => Some(TestVariant::Service),
            Workload::Action => Some(TestVariant::Action),
            _ => None,
        }
    }
}

/// What the cell exercises.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Single-node example pair (talker/listener, server/client).
    Example,
    /// Entry-pkg workspace (`nros ws` shape, launch-driven).
    Workspace,
    /// nano-ros node against a REAL ROS 2 peer.
    Interop,
    /// Declarative bridge chains.
    Bridge,
}

/// Coverage tier — the load-bearing part of the table.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Tier {
    /// A runtime e2e lane exists (or must exist — the consumer runs it).
    Runtime,
    /// Compiles/links as a build-stage fixture; no runtime lane yet. The
    /// string says what unlocks it.
    BuildOnly(&'static str),
    /// Deliberately unsupported / not applicable. The string is the
    /// recorded reason (audit E5).
    CarveOut(&'static str),
}

/// One cell of the matrix.
///
/// Issue 0327 — the ROS **edition** (ARCHITECTURE §2's third axis) is
/// deliberately NOT a `Cell` field: it is a PER-RUN GLOBAL, not a per-cell
/// dimension. A run targets one edition via `NROS_ROS_EDITION`
/// (`ros_env::test_edition()`, default `jazzy`) and executes the whole matrix
/// against it — there is no "jazzy pubsub" vs "humble pubsub" as distinct cells
/// in one run, so folding edition into `Cell` would multiply every row ×N
/// editions with no per-cell distinction. Edition coverage (which editions run,
/// and the humble/iron `rmw_zenoh_cpp` carve-out) is documented in
/// `examples/README.md`'s coverage matrix + ARCHITECTURE §2, not enumerated
/// here. (The prior silence about which kind of axis edition was is what let the
/// per-cell `ros_editions_*` files drift from RFC-0051.)
#[derive(Copy, Clone, Debug)]
pub struct Cell {
    pub platform: PlatformId,
    pub lang: Lang,
    pub rmw: Rmw,
    pub workload: Workload,
    pub kind: Kind,
    pub tier: Tier,
}

const fn cell(
    platform: PlatformId,
    lang: Lang,
    rmw: Rmw,
    workload: Workload,
    kind: Kind,
    tier: Tier,
) -> Cell {
    Cell {
        platform,
        lang,
        rmw,
        workload,
        kind,
        tier,
    }
}

/// A thing the test-tier tooling can treat as a matrix cell, whichever list it
/// came from (issue 0352 / phase-324). `matrix::CELLS` holds baked
/// self-contained cells; `crate::interop::CELLS` holds interop/bridge cells in
/// their richer shape. Both expose their underlying [`Cell`] so `ci_lane`
/// selection, coordinate derivation and coverage all iterate one type without
/// caring which list a cell belongs to.
pub trait TestCell {
    fn cell(&self) -> &Cell;
}

impl TestCell for Cell {
    fn cell(&self) -> &Cell {
        self
    }
}

// Shorthand used by the seed table below.
use Kind::*;
use Lang::*;
use PlatformId::*;
use Rmw::*;
use Tier::*;
use Workload::*;

/// W6 (2026-07-18) decided each cyclone/xrce-on-RTOS gap cell. Implement-
/// worthy cells (native rust cyclone service/action; threadx C cyclone
/// service/action; threadx C++ cyclone pubsub) are tracked in issue #233
/// and stay BuildOnly until wired; the rest are firm CarveOuts.
const CYCLONE_RUST_RTOS_CARVE: &str =
    "cyclone-on-RTOS is C/C++ only; pure-rust image has no cyclone backend symbol (#163 class)";
const XRCE_RTOS_CARVE: &str =
    "no XRCE agent-locator bake off Zephyr; rust-XRCE-on-bare-RTOS is not a shipped config";

/// The seed table (phase-295 W1): codifies the 2026-07-17 survey's REAL
/// coverage. Every pre-295 runtime lane appears as a `Runtime` cell;
/// every known gap as `BuildOnly`/`CarveOut` with its reason. The matrix
/// consumers (W3) iterate this; the fixture coverage gate cross-checks
/// it against `examples/fixtures.toml`.
#[rustfmt::skip]
pub const CELLS: &[Cell] = &[
    // ── Example kind: the classic pubsub/service/action pairs ──────────
    // Linux host (ephemeral isolation; all three RMWs have runtime lanes).
    cell(Linux, Rust, Zenoh,      Pubsub,  Example, Runtime),
    cell(Linux, C,    Zenoh,      Pubsub,  Example, Runtime),
    cell(Linux, Cpp,  Zenoh,      Pubsub,  Example, Runtime),
    cell(Linux, Rust, Zenoh,      Service, Example, Runtime),
    cell(Linux, C,    Zenoh,      Service, Example, Runtime),
    cell(Linux, Cpp,  Zenoh,      Service, Example, Runtime),
    cell(Linux, Rust, Zenoh,      Action,  Example, Runtime),
    cell(Linux, C,    Zenoh,      Action,  Example, Runtime),
    cell(Linux, Cpp,  Zenoh,      Action,  Example, Runtime),
    cell(Linux, Rust, Cyclonedds, Pubsub,  Example, Runtime),
    cell(Linux, C,    Cyclonedds, Pubsub,  Example, Runtime),
    cell(Linux, Cpp,  Cyclonedds, Pubsub,  Example, Runtime),
    cell(Linux, C,    Cyclonedds, Service, Example, Runtime),
    cell(Linux, Cpp,  Cyclonedds, Service, Example, Runtime),
    // issue #233 cell 1 — proven: rust cyclone service pair delivers
    // (test_native_cyclonedds_rust_service).
    cell(Linux, Rust, Cyclonedds, Service, Example, Runtime),
    cell(Linux, C,    Cyclonedds, Action,  Example, Runtime),
    cell(Linux, Cpp,  Cyclonedds, Action,  Example, Runtime),
    // issue #234 — RESOLVED: rust cyclone action pair delivers the order-10
    // Fibonacci result (test_native_cyclonedds_rust_action). The action's
    // `register_protocol_types` now registers the `action_msgs` descriptors
    // through the generic `nros_rmw::register_type_descriptor` seam instead of
    // the cfg-gated named-backend call that compiled out of the example build.
    cell(Linux, Rust, Cyclonedds, Action,  Example, Runtime),
    cell(Linux, C,    Xrce,       Pubsub,  Example, Runtime),
    cell(Linux, Rust, Xrce,       Pubsub,  Example, Runtime),
    cell(Linux, Cpp,  Xrce,       Pubsub,  Example, Runtime),
    cell(Linux, C,    Xrce,       Service, Example, Runtime),
    cell(Linux, Rust, Xrce,       Service, Example, Runtime),
    cell(Linux, Cpp,  Xrce,       Service, Example, Runtime),
    cell(Linux, C,    Xrce,       Action,  Example, Runtime),
    cell(Linux, Rust, Xrce,       Action,  Example, Runtime),
    cell(Linux, Cpp,  Xrce,       Action,  Example, Runtime),

    // Zephyr on QEMU mps2/an385 (Cortex-M3) — phase-337 W2.d. The point of
    // these rows is everything the native_sim rows below CANNOT say: 32-bit
    // pointers, Zephyr's own in-kernel IP stack, and a real ethernet driver
    // (`eth_smsc911x` against QEMU's lan9118). Proven end to end in W2.b — the
    // image boots, takes 10.0.2.15 from the driver, and publishes over a zenoh
    // session to a host router through SLIRP.
    //
    // NO RUST ROW, deliberately, and NOT as a BuildOnly: the pinned
    // `zephyr-lang-rust` cannot compile the `zephyr` crate for any board whose
    // devicetree has gpio nodes (issue 0432), so a rust cell here would not
    // build either, and `BuildOnly` promises that it does. An absent row with
    // the reason recorded beats a tier that lies — see this module's header.
    //
    // Zenoh only: cyclone and xrce on this board are untried, and the honest
    // record of "nobody has attempted it" is no row, not a guess.
    cell(ZephyrQemuCortexM, C,   Zenoh, Pubsub,  Example, Runtime),
    cell(ZephyrQemuCortexM, Cpp, Zenoh, Pubsub,  Example, Runtime),
    // phase-346 W3 — Rust on a REAL Zephyr board, unblocked by patching issue
    // 0432's two upstream defects. Until then the `zephyr` crate could not
    // compile for any board whose devicetree has gpio nodes, which is every
    // real one; native_sim has none, which is the only reason it went unseen.
    cell(ZephyrQemuCortexM, Rust, Zenoh, Pubsub,  Example, Runtime),
    cell(ZephyrQemuCortexM, C,   Zenoh, Service, Example,
         BuildOnly("phase-337 W2.f — the runner reproduces W2.b's manual setup \
                    (SLIRP + a host zenoh router on 7456) for pubsub only; \
                    service needs the peer half of that harness")),
    cell(ZephyrQemuCortexM, Cpp, Zenoh, Service, Example,
         BuildOnly("phase-337 W2.f — same runner gap as the C service row")),
    cell(ZephyrQemuCortexM, C,   Zenoh, Action,  Example,
         BuildOnly("phase-337 W2.f — same runner gap as the C service row")),
    cell(ZephyrQemuCortexM, Cpp, Zenoh, Action,  Example,
         BuildOnly("phase-337 W2.f — same runner gap as the C service row")),

    // Zephyr native_sim — zenoh + cyclone + xrce, all three langs
    // (the zephyr.rs families; W4 bakes: cyclone domains 22–30, xrce
    // agents 2400+ — `alloc::{domain_of,xrce_agent_port_of}`).
    cell(ZephyrNativeSim, Rust, Zenoh,      Pubsub,  Example, Runtime),
    cell(ZephyrNativeSim, C,    Zenoh,      Pubsub,  Example, Runtime),
    cell(ZephyrNativeSim, Cpp,  Zenoh,      Pubsub,  Example, Runtime),
    cell(ZephyrNativeSim, Rust, Zenoh,      Service, Example, Runtime),
    cell(ZephyrNativeSim, C,    Zenoh,      Service, Example, Runtime),
    cell(ZephyrNativeSim, Cpp,  Zenoh,      Service, Example, Runtime),
    cell(ZephyrNativeSim, Rust, Zenoh,      Action,  Example, Runtime),
    cell(ZephyrNativeSim, C,    Zenoh,      Action,  Example, Runtime),
    cell(ZephyrNativeSim, Cpp,  Zenoh,      Action,  Example, Runtime),
    cell(ZephyrNativeSim, Rust, Cyclonedds, Pubsub,  Example, Runtime),
    cell(ZephyrNativeSim, C,    Cyclonedds, Pubsub,  Example, Runtime),
    cell(ZephyrNativeSim, Cpp,  Cyclonedds, Pubsub,  Example, Runtime),
    cell(ZephyrNativeSim, Rust, Cyclonedds, Service, Example, Runtime),
    cell(ZephyrNativeSim, C,    Cyclonedds, Service, Example, Runtime),
    cell(ZephyrNativeSim, Cpp,  Cyclonedds, Service, Example, Runtime),
    cell(ZephyrNativeSim, Rust, Cyclonedds, Action,  Example, Runtime),
    cell(ZephyrNativeSim, C,    Cyclonedds, Action,  Example, Runtime),
    cell(ZephyrNativeSim, Cpp,  Cyclonedds, Action,  Example, Runtime),
    cell(ZephyrNativeSim, Rust, Xrce,       Pubsub,  Example, Runtime),
    cell(ZephyrNativeSim, C,    Xrce,       Pubsub,  Example, Runtime),
    cell(ZephyrNativeSim, Cpp,  Xrce,       Pubsub,  Example, Runtime),
    cell(ZephyrNativeSim, Rust, Xrce,       Service, Example, Runtime),
    cell(ZephyrNativeSim, C,    Xrce,       Service, Example, Runtime),
    cell(ZephyrNativeSim, Cpp,  Xrce,       Service, Example, Runtime),
    cell(ZephyrNativeSim, Rust, Xrce,       Action,  Example, Runtime),
    cell(ZephyrNativeSim, C,    Xrce,       Action,  Example, Runtime),
    cell(ZephyrNativeSim, Cpp,  Xrce,       Action,  Example, Runtime),

    // FreeRTOS / NuttX-arm / ThreadX-linux — the rtos_e2e 3×3 zenoh block.
    cell(FreertosMps2, Rust, Zenoh, Pubsub,  Example, Runtime),
    cell(FreertosMps2, C,    Zenoh, Pubsub,  Example, Runtime),
    cell(FreertosMps2, Cpp,  Zenoh, Pubsub,  Example, Runtime),
    cell(FreertosMps2, Rust, Zenoh, Service, Example, Runtime),
    cell(FreertosMps2, C,    Zenoh, Service, Example, Runtime),
    cell(FreertosMps2, Cpp,  Zenoh, Service, Example, Runtime),
    cell(FreertosMps2, Rust, Zenoh, Action,  Example, Runtime),
    cell(FreertosMps2, C,    Zenoh, Action,  Example, Runtime),
    cell(FreertosMps2, Cpp,  Zenoh, Action,  Example, Runtime),
    cell(FreertosMps2, Rust, Cyclonedds, Pubsub, Example,
         BuildOnly("fixture retired in phase-220.C (cmake-bridge removed); \
                    freertos_qemu.rs lanes #[ignore]d pending the 214.S.5.b \
                    pure-cargo BSP gate — issue #233 tracks restore-vs-carve")),
    cell(FreertosMps2, Rust, Xrce,       Pubsub, Example, CarveOut(XRCE_RTOS_CARVE)),

    cell(NuttxArm, Rust, Zenoh, Pubsub,  Example, Runtime),
    cell(NuttxArm, C,    Zenoh, Pubsub,  Example, Runtime),
    cell(NuttxArm, Cpp,  Zenoh, Pubsub,  Example, Runtime),
    cell(NuttxArm, Rust, Zenoh, Service, Example, Runtime),
    cell(NuttxArm, C,    Zenoh, Service, Example, Runtime),
    cell(NuttxArm, Cpp,  Zenoh, Service, Example, Runtime),
    cell(NuttxArm, Rust, Zenoh, Action,  Example, Runtime),
    cell(NuttxArm, C,    Zenoh, Action,  Example, Runtime),
    cell(NuttxArm, Cpp,  Zenoh, Action,  Example, Runtime),
    cell(NuttxArm, Rust, Cyclonedds, Pubsub, Example, CarveOut(CYCLONE_RUST_RTOS_CARVE)),
    cell(NuttxArm, Rust, Xrce,       Pubsub, Example, CarveOut(XRCE_RTOS_CARVE)),

    cell(ThreadxLinux, Rust, Zenoh, Pubsub,  Example, Runtime),
    cell(ThreadxLinux, C,    Zenoh, Pubsub,  Example, Runtime),
    cell(ThreadxLinux, Cpp,  Zenoh, Pubsub,  Example, Runtime),
    cell(ThreadxLinux, Rust, Zenoh, Service, Example, Runtime),
    cell(ThreadxLinux, C,    Zenoh, Service, Example, Runtime),
    cell(ThreadxLinux, Cpp,  Zenoh, Service, Example, Runtime),
    cell(ThreadxLinux, Rust, Zenoh, Action,  Example, Runtime),
    cell(ThreadxLinux, C,    Zenoh, Action,  Example, Runtime),
    cell(ThreadxLinux, Cpp,  Zenoh, Action,  Example, Runtime),
    // threadx-linux cyclone: C pubsub pair proven (native_api #215 lane);
    // service/action fixtures build but have no runtime lane.
    cell(ThreadxLinux, C,   Cyclonedds, Pubsub,  Example, Runtime),
    // issue #233 cell 3 — threadx C cyclone service proven (test_threadx_linux_cyclonedds_service).
    cell(ThreadxLinux, C,   Cyclonedds, Service, Example, Runtime),
    // issue #233 cell 3 — threadx C cyclone action proven (test_threadx_linux_cyclonedds_action).
    cell(ThreadxLinux, C,   Cyclonedds, Action,  Example, Runtime),
    // issue #233 cell 4 — threadx C++ cyclone pubsub proven (test_threadx_linux_cyclonedds_cpp_talker_to_native_listener).
    cell(ThreadxLinux, Cpp, Cyclonedds, Pubsub,  Example, Runtime),

    // ThreadX riscv64 — pubsub + service runtime (all three langs; rtos_e2e
    // runs the full lang fan-out). Action examples + builders EXIST in all
    // three langs but were deliberately dropped from the run matrix in
    // phase-182.5 (action is the wall-clock critical path — see rtos_e2e.rs);
    // cyclone two-QEMU pubsub pairs proven (#214).
    cell(ThreadxRiscv64, Rust, Zenoh, Pubsub,  Example, Runtime),
    cell(ThreadxRiscv64, C,    Zenoh, Pubsub,  Example, Runtime),
    cell(ThreadxRiscv64, Cpp,  Zenoh, Pubsub,  Example, Runtime),
    cell(ThreadxRiscv64, Rust, Zenoh, Service, Example, Runtime),
    cell(ThreadxRiscv64, C,    Zenoh, Service, Example, Runtime),
    cell(ThreadxRiscv64, Cpp,  Zenoh, Service, Example, Runtime),
    cell(ThreadxRiscv64, Rust, Zenoh, Action, Example,
         BuildOnly("dropped from the action run matrix in 182.5 (wall-clock); examples + rtos_e2e builders exist")),
    cell(ThreadxRiscv64, C,    Zenoh, Action, Example,
         BuildOnly("dropped from the action run matrix in 182.5 (wall-clock); examples + rtos_e2e builders exist")),
    cell(ThreadxRiscv64, Cpp,  Zenoh, Action, Example,
         BuildOnly("dropped from the action run matrix in 182.5 (wall-clock); examples + rtos_e2e builders exist")),
    // issue 0680 — per-thread `errno`. Not a messaging workload: the fixture
    // spawns two tasks and asserts one cannot see the other's `errno`. It is a
    // cell so the fixture has an owner that RUNS it; before the fix it fails
    // with `FAIL shared errno`, which is what makes it worth a coordinate.
    cell(ThreadxRiscv64, C,    Zenoh, Errno, Example, Runtime),
    cell(ThreadxRiscv64, C,    Cyclonedds, Pubsub, Example, Runtime),
    cell(ThreadxRiscv64, Rust, Cyclonedds, Pubsub, Example, Runtime),
    // issue #235 — the cpp cyclone riscv64 fixtures existed (distinct
    // identity per node); the two-QEMU lane
    // (test_threadx_riscv64_cyclonedds_two_qemu_cpp_pubsub) now consumes them.
    cell(ThreadxRiscv64, Cpp,  Cyclonedds, Pubsub, Example, Runtime),

    // NuttX riscv — the C talker example has a runtime lane
    // (c_riscv_nuttx_e2e); rust/cpp have NO standalone pubsub examples —
    // their riscv coverage is the realtime-tiers WORKSPACE lanes (rows
    // below), so don't claim Example-Runtime here.
    cell(NuttxRiscv, C,    Zenoh, Pubsub, Example, Runtime),
    cell(NuttxRiscv, Cpp,  Zenoh, Pubsub, Example,
         CarveOut("no standalone cpp pubsub example on rv-virt; runtime coverage rides the realtime-tiers workspace lane")),
    cell(NuttxRiscv, Rust, Zenoh, Pubsub, Example,
         CarveOut("no standalone rust pubsub example on rv-virt; runtime coverage rides the realtime-tiers workspace lane")),

    // ESP32 — rust pubsub runtime under the Espressif QEMU fork (plus the
    // workspace-entry lane in esp32_emulator.rs); service/action examples
    // are NOT authored (example set is talker/listener only). C/C++
    // build-only.
    cell(Esp32Qemu, Rust, Zenoh, Pubsub,  Example, Runtime),
    cell(Esp32Qemu, Rust, Zenoh, Service, Example,
         CarveOut("service/action examples not authored on esp32-qemu (talker/listener set only)")),
    cell(Esp32Qemu, Rust, Zenoh, Action,  Example,
         CarveOut("service/action examples not authored on esp32-qemu (talker/listener set only)")),
    cell(Esp32Qemu, C,    Zenoh, Pubsub,  Example,
         BuildOnly("IDF C runtime lane pending (espressif qemu fork drives rust only today)")),
    cell(Esp32Qemu, Cpp,  Zenoh, Pubsub,  Example,
         BuildOnly("IDF C++ runtime lane pending")),

    // Bare-metal RTIC (QEMU MPS2) — pubsub-only demo set by design.
    cell(QemuBaremetal, Rust, Zenoh, Pubsub, Example, Runtime),
    cell(QemuBaremetal, Rust, Zenoh, Service, Example,
         CarveOut("rtic demo set is pubsub-only by design (phase-289 scope)")),
    cell(QemuBaremetal, Rust, Zenoh, Action, Example,
         CarveOut("rtic demo set is pubsub-only by design (phase-289 scope)")),

    // phase-337 W7.a — the three `Stm32F4` BuildOnly cells left with the board.
    // They were `BuildOnly`, i.e. ZERO Runtime, so the removal costs no
    // coverage: `check-board-tiers` requires a Runtime cell for tier 1/2 and
    // this platform could never have one (no hardware in the rack, no QEMU
    // model of the F4 MAC). The board is now
    // `book/src/porting/stm32f4-out-of-tree.md`.

    // FVP — cyclone runtime (license-gated at run time), cpp + rust.
    // Issue 0232 / phase-320 W1.a — these were `Runtime`, which the lane can
    // never satisfy: the Base_RevC AEMv8-R model is license-walled
    // (`[gated.arm-fvp]` in nros-sdk-index.toml, user-supplied via ARM_FVP_DIR),
    // so `fvp_smoke` / `fvp_runtime_ws` skip on EVERY CI and dev host. Claiming
    // Runtime here is the exact shape of 0232's false green — a lane that always
    // skipped, so four walls "shipped invisible and were found by the ASI
    // consumer". A gap reads as a gap; an overclaim reads as confidence.
    //
    // The maintainer-run runtime gate still exists and still matters
    // (`just zephyr verify-fvp-runtime`); it is simply not coverage this matrix
    // can promise. Note the runtime-verified FVP artifact is the two-tier
    // workspace Entry, not these example cells.
    cell(Fvp, Cpp,  Cyclonedds, Pubsub, Example,
         BuildOnly("license-gated model; runtime needs ARM_FVP_DIR and is maintainer-run \
                    via `just zephyr verify-fvp-runtime` (phase-298)")),
    cell(Fvp, Rust, Cyclonedds, Pubsub, Example,
         BuildOnly("license-gated model; runtime needs ARM_FVP_DIR and is maintainer-run \
                    via `just zephyr verify-fvp-runtime` (phase-298)")),
    cell(Fvp, Cpp,  Zenoh,      Pubsub, Example,
         CarveOut("zenoh-pico needs POSIX API the FVP board conf doesn't enable (#217)")),

    // ── Workspace kind (Entry-pkg lanes; native-heavy today) ──────────
    cell(Linux, Rust,  Zenoh, EntryPubsub, Workspace, Runtime),
    cell(Linux, C,     Zenoh, EntryPubsub, Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, EntryPubsub, Workspace, Runtime),
    cell(Linux, Mixed, Zenoh, EntryPubsub, Workspace, Runtime),
    // phase-331 W4 (RFC-0066) — the RMW AXIS over the language workspaces.
    // Before this the workspace rows were 85/87 zenoh: the RMW seam was
    // exercised by single-node examples and essentially never by a workspace.
    // `mixed` is deliberately absent — its value is the language seam, not the
    // RMW seam. The rust cells already existed; only c/cpp were missing.
    cell(Linux, Rust, Cyclonedds, EntryPubsub, Workspace, Runtime),
    cell(Linux, C,    Cyclonedds, EntryPubsub, Workspace, Runtime),
    cell(Linux, Cpp,  Cyclonedds, EntryPubsub, Workspace, Runtime),
    cell(Linux, Rust, Xrce,       EntryPubsub, Workspace, Runtime),
    cell(Linux, C,    Xrce,       EntryPubsub, Workspace, Runtime),
    cell(Linux, Cpp,  Xrce,       EntryPubsub, Workspace, Runtime),
    cell(ZephyrNativeSim, Rust,  Zenoh, EntryPubsub, Workspace, Runtime),
    cell(ZephyrNativeSim, C,     Zenoh, EntryPubsub, Workspace, Runtime),
    cell(ZephyrNativeSim, Cpp,   Zenoh, EntryPubsub, Workspace, Runtime),
    cell(ZephyrNativeSim, Mixed, Zenoh, EntryPubsub, Workspace, Runtime),
    cell(FreertosMps2, C,    Zenoh, EntryPubsub, Workspace, Runtime),
    cell(FreertosMps2, Cpp,  Zenoh, EntryPubsub, Workspace, Runtime),
    cell(FreertosMps2, Rust, Zenoh, EntryPubsub, Workspace, Runtime),
    // phase-370 W3 — the freertos family's first cells that run without QEMU.
    // The POSIX simulator is a host process, so `Runtime` here costs a process
    // spawn rather than an emulator, and the RMW is the same host CycloneDDS
    // the `Linux` rows use. No Rust row: the POSIX port has no Rust entry shape
    // of its own and the board carries no Rust board crate.
    cell(FreertosPosix, C,   Cyclonedds, EntryPubsub, Workspace, Runtime),
    cell(FreertosPosix, Cpp, Cyclonedds, EntryPubsub, Workspace, Runtime),
    // phase-372 — the S32Z270 Cortex-R52 bundle's workspace row
    // (`workspace-cpp-s32z270-freertos`). It borrows the `freertos` platform
    // token for the family lane; MPS2 owns that platform's Runtime cells, and
    // this board has no witness of its own, so it is BuildOnly rather than a
    // promise CI runs. The string is what unlocks it.
    cell(FreertosMps2, Cpp, Cyclonedds, EntryPubsub, Workspace,
         BuildOnly("phase-372 — S32Z270 is link-complete only: the NXP GIC port, \
                    RTD NETC driver and PBcfg are consumer-provisioned seams \
                    (autoware-safety-island), so no in-tree runner can boot it. \
                    Unlocks when a hardware or simulator witness exists")),
    cell(NuttxArm, C,    Zenoh, EntryPubsub, Workspace, Runtime),
    // Corrected during the phase-295 W3.b entry consolidation: the seed
    // table marked the nuttx-arm C++ and all three nuttx-riscv EntryPubsub
    // rows `Runtime`, but no EntryPubsub fixture or lane exists at those
    // coordinates — the only nuttx workspace rows besides the C arm entry
    // are the REALTIME-TIERS entries (the fixtures.toml realtime rows +
    // workspace-rust-nuttx-riscv-realtime), which satisfied
    // the (platform, lang) coverage gate and masked the gap. The riscv C
    // runtime proof that exists is the STANDALONE talker example
    // (c_riscv_nuttx_e2e — the `(NuttxRiscv, C, Pubsub, Example)` cell).
    cell(NuttxArm, Cpp,  Zenoh, EntryPubsub, Workspace,
         BuildOnly("no nuttx-arm C++ EntryPubsub fixture/lane; only the RT-tiers C++ \
                    workspace builds at this coordinate — phase-295 W3.b finding, W6 wires it")),
    cell(NuttxRiscv, C,   Zenoh, EntryPubsub, Workspace,
         BuildOnly("no nuttx-riscv C EntryPubsub workspace fixture/lane (RT-tiers only; \
                    the standalone talker example is the riscv C runtime proof) — \
                    phase-295 W3.b finding, W6 wires it")),
    cell(NuttxRiscv, Cpp, Zenoh, EntryPubsub, Workspace,
         BuildOnly("no nuttx-riscv C++ EntryPubsub workspace fixture/lane (RT-tiers only) \
                    — phase-295 W3.b finding, W6 wires it")),
    cell(ThreadxLinux, Rust,  Zenoh, EntryPubsub, Workspace, Runtime),
    cell(ThreadxLinux, C,     Zenoh, EntryPubsub, Workspace, Runtime),
    cell(ThreadxLinux, Cpp,   Zenoh, EntryPubsub, Workspace, Runtime),
    cell(ThreadxLinux, Mixed, Zenoh, EntryPubsub, Workspace, Runtime),
    cell(FreertosMps2, Mixed, Zenoh, EntryPubsub, Workspace, Runtime),
    cell(NuttxArm,     Rust,  Zenoh, EntryPubsub, Workspace, Runtime),
    // See the nuttx-riscv correction above — the rust riscv workspace row
    // is realtime-only too (workspace-rust-nuttx-riscv-realtime); no
    // EntryPubsub image or lane exists. phase-295 W3.b finding.
    cell(NuttxRiscv,   Rust,  Zenoh, EntryPubsub, Workspace,
         BuildOnly("no nuttx-riscv rust EntryPubsub workspace fixture/lane (RT-tiers \
                    only) — phase-295 W3.b finding, W6 wires it")),
    cell(Esp32Qemu,    Rust, Zenoh, EntryPubsub, Workspace, Runtime),

    // Workspace feature workloads (native + zephyr today; per-lang rows
    // mirror the ws-* families).
    cell(Linux, C,     Zenoh, CustomMsg, Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, CustomMsg, Workspace, Runtime),
    // Corrected during the phase-295 W3.b consolidation: the seed table
    // marked native rust CustomMsg/Qos `Runtime`, but no fixtures.toml row
    // builds `ws-{custom-msg,qos}-rust`'s `native_entry` and no test
    // consumes it (the C files' "C projection of the Rust demo" prose
    // described the WORKSPACE, not a lane; ws-qos-rust's only runtime lane
    // is the zephyr image). Single-entry natives also hit issue 0096
    // (in-process pub→sub never delivers), so wiring them needs split
    // talker/listener entries first — issue #233.
    cell(Linux, Rust,  Zenoh, CustomMsg, Workspace,
         BuildOnly("ws-custom-msg-rust native_entry has no fixture row or runtime lane \
                    (needs an 0096 two-entry split) — phase-295 W3.b finding, W6 wires it")),
    cell(Linux, Mixed, Zenoh, CustomMsg, Workspace, Runtime),
    cell(Linux, C,     Zenoh, Logging,   Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, Logging,   Workspace, Runtime),
    // Added during the phase-295 W3.b consolidation: the rust + mixed
    // logging lanes existed (tests/{,mixed_}logging_workspace_e2e.rs,
    // phase-263 A5) but the seed table never modeled them.
    cell(Linux, Rust,  Zenoh, Logging,   Workspace, Runtime),
    cell(Linux, Mixed, Zenoh, Logging,   Workspace, Runtime),
    cell(Linux, C,     Zenoh, Qos,       Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, Qos,       Workspace, Runtime),
    // See the CustomMsg rust row above — same phase-295 W3.b correction.
    cell(Linux, Rust,  Zenoh, Qos,       Workspace,
         BuildOnly("ws-qos-rust native_entry has no fixture row or runtime lane (only \
                    the zephyr image is consumed) — phase-295 W3.b finding, W6 wires it")),
    cell(Linux, Mixed, Zenoh, Qos,       Workspace, Runtime),
    cell(Linux, C,     Zenoh, Params,    Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, Params,    Workspace, Runtime),
    cell(Linux, Rust,  Zenoh, Params,    Workspace, Runtime),
    cell(Linux, C,     Zenoh, Lifecycle, Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, Lifecycle, Workspace, Runtime),
    cell(Linux, Rust,  Zenoh, Lifecycle, Workspace, Runtime),
    cell(Linux, C,     Zenoh, Safety,    Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, Safety,    Workspace, Runtime),
    cell(Linux, Rust,  Zenoh, Safety,    Workspace, Runtime),
    // phase-306 W4 (issue 0255) — launch/model remap + `~` private name hits
    // the wire remapped. Rust only: the C/C++ `nros_cpp_declare_remap` path is
    // emitter-unit-tested (W3); a runtime C/C++ cell is residual.
    cell(Linux, Rust,  Zenoh, Remap,     Workspace, Runtime),
    cell(ZephyrNativeSim, Rust, Zenoh, Params,    Workspace, Runtime),
    cell(ZephyrNativeSim, Rust, Zenoh, Lifecycle, Workspace, Runtime),
    cell(ZephyrNativeSim, Rust, Zenoh, Qos,       Workspace, Runtime),
    cell(ZephyrNativeSim, Rust, Zenoh, Safety,    Workspace, Runtime),

    // Realtime tiers + multihost.
    cell(Linux, Rust, Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(Linux, C,    Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(Linux, Cpp,  Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(ZephyrNativeSim, Rust, Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(ZephyrNativeSim, C,    Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(ZephyrNativeSim, Cpp,  Zenoh, RealtimeTiers, Workspace, Runtime),
    // Corrected during the phase-295 W4 re-bake: the realtime_tiers_e2e
    // consumer has ALWAYS run nuttx-arm {c,rust}, nuttx-riscv {rust,c} and
    // freertos c cells (fixtures.toml rows existed for each), but the seed
    // table only modeled the cpp rows — the (platform, lang) coverage gate
    // was satisfied by other workspace rows and masked the gap. Modeled so
    // the allocator derives every baked realtime port.
    cell(NuttxArm,   Cpp,  Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(NuttxArm,   C,    Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(NuttxArm,   Rust, Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(NuttxRiscv, Cpp,  Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(NuttxRiscv, C,    Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(NuttxRiscv, Rust, Zenoh, RealtimeTiers, Workspace, Runtime),
    // issue 0636 gap 2 — the Rust arm. `run_tiers_entry` was exported and
    // reachable and called by nothing, so the boot-tier fix on this board was
    // reasoned onto Rust rather than measured. This cell measures it.
    cell(FreertosMps2, Rust, Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(FreertosMps2, Cpp, Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(FreertosMps2, C,   Zenoh, RealtimeTiers, Workspace, Runtime),
    // phase-297 W5 (RFC-0053) — ThreadX multi-tier run_tiers acceptance:
    // hosted simulation (pthread-backed ThreadX, host binary + NSOS host
    // sockets), port 9091 = port_of(ThreadxLinux, Rust, RealtimeTiers).
    cell(ThreadxLinux, Rust, Zenoh, RealtimeTiers, Workspace, Runtime),
    cell(Linux, Rust,  Zenoh, Multihost, Workspace, Runtime),
    cell(Linux, C,     Zenoh, Multihost, Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, Multihost, Workspace, Runtime),
    cell(Linux, Mixed, Zenoh, Multihost, Workspace, Runtime),
    // The embedded multihost lane is the RUST robot1 zephyr image (276 W6);
    // corrected from Cpp during the phase-295 W3.b consolidation.
    cell(ZephyrNativeSim, Rust, Zenoh, Multihost, Workspace, Runtime),

    // Cross-process service/action roundtrips (phase-263 A1/A4; issue 0096
    // forces the two-process topology) — tests/roundtrip_xprocess_e2e.rs.
    cell(Linux, Rust,  Zenoh, Service, Workspace, Runtime),
    cell(Linux, C,     Zenoh, Service, Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, Service, Workspace, Runtime),
    cell(Linux, Mixed, Zenoh, Service, Workspace, Runtime),
    cell(Linux, Rust,  Zenoh, Action,  Workspace, Runtime),
    cell(Linux, C,     Zenoh, Action,  Workspace, Runtime),
    cell(Linux, Cpp,   Zenoh, Action,  Workspace, Runtime),
    cell(Linux, Mixed, Zenoh, Action,  Workspace, Runtime),

    // Workspace RMW variants (thin today: 80/82 rows are zenoh — issue #233).

    // ── Interop & Bridge kinds — issue 0352 / phase-324 ────────────────
    // These cells carry a nano side that is BUILT plus an ephemeral PEER and a
    // DIRECTION, a shape `Cell` cannot express. They live in `crate::interop`
    // (`interop::CELLS`) in that formulation, joined to their build + test
    // recipe by a `Binding` and gated by `tests/matrix_fixture_coverage.rs`.
    // `matrix::CELLS` is baked/self-contained only.

    // ── uORB (PX4-SITL) — issue 0341 ───────────────────────────────────
    // uORB is a declared RMW (ARCHITECTURE §2) with a real crate
    // (nros-rmw-uorb) + example (packages/testing/nros-px4-register-check), but its runtime lane
    // is a PX4-SITL build no CI runner here provides. Expressible + carved out,
    // so the gap is visible rather than inexpressible.
    cell(Px4, Cpp, Uorb, Pubsub, Example,
         CarveOut("uORB runs only inside a PX4-SITL build (just px4 …); no CI runner \
                   builds SITL. `packages/testing/nros-px4-register-check` is the source of truth.")),
];

/// Runtime cells only — what the matrix consumers iterate.
pub fn runtime_cells() -> impl Iterator<Item = &'static Cell> {
    CELLS.iter().filter(|c| matches!(c.tier, Tier::Runtime))
}

/// The five RFC-0051 "matrix consumer" test files (phase-329 W1). Each Runtime
/// cell a W1 consumer runs is CLAIMED by exactly one of these; the consumer's
/// test DERIVES its cases from `CELLS.filter(|c| w1_consumer_of(c) == Some(..))`
/// instead of hand-listing `Cell{}` literals, so adding a cell to `CELLS` makes
/// it run in the right consumer.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum W1Consumer {
    /// `tests/multihost_e2e.rs` — every `Multihost` cell.
    Multihost,
    /// `tests/roundtrip_xprocess_e2e.rs` — native workspace `Service`/`Action`.
    Roundtrip,
    /// `tests/realtime_tiers_e2e.rs` — every `RealtimeTiers` cell.
    RealtimeTiers,
    /// `tests/entry_e2e.rs` — the RTOS `EntryPubsub` subset + zephyr-rust
    /// feature entries.
    Entry,
    /// `tests/workspace_features_e2e.rs` — native feature workspaces.
    WorkspaceFeatures,
}

/// Which W1 consumer owns this cell's runtime lane, if any. THE authoritative
/// claim partition — disjoint by construction (one match arm each). `None` =
/// covered by a not-yet-converted platform-e2e file (phase-329 W4) or a
/// non-matrix suite, NOT a W1 consumer's responsibility.
///
/// `Multihost`, `RealtimeTiers`, and workspace `Service`/`Action` are FULLY
/// owned (G5 asserts every Runtime cell of them is claimed). `EntryPubsub` and
/// the native feature workloads are SHARED with W4 files, so only the subset a
/// W1 consumer actually runs is claimed here.
pub fn w1_consumer_of(cell: &Cell) -> Option<W1Consumer> {
    use W1Consumer::*;
    match (cell.platform, cell.lang, cell.workload, cell.kind) {
        // multihost_e2e — every Multihost cell.
        (_, _, Workload::Multihost, _) => Some(Multihost),
        // realtime_tiers_e2e — every RealtimeTiers cell.
        (_, _, Workload::RealtimeTiers, _) => Some(RealtimeTiers),
        // roundtrip_xprocess_e2e — workspace service/action (all native today).
        (_, _, Workload::Service | Workload::Action, Kind::Workspace) => Some(Roundtrip),
        // entry_e2e — the RTOS EntryPubsub subset it actually boots (threadx /
        // freertos / zephyr are C/C++/mixed; nuttx-arm is C + rust). The rust
        // EntryPubsub cells on threadx/freertos/zephyr, nuttx-riscv, esp32, and
        // native EntryPubsub are OTHER consumers — deliberately not claimed.
        (PlatformId::ThreadxLinux, Lang::C | Lang::Cpp | Lang::Mixed, Workload::EntryPubsub, _) => {
            Some(Entry)
        }
        (PlatformId::FreertosMps2, Lang::C | Lang::Cpp | Lang::Mixed, Workload::EntryPubsub, _) => {
            Some(Entry)
        }
        (
            PlatformId::ZephyrNativeSim,
            Lang::C | Lang::Cpp | Lang::Mixed,
            Workload::EntryPubsub,
            _,
        ) => Some(Entry),
        (PlatformId::NuttxArm, Lang::C | Lang::Rust, Workload::EntryPubsub, _) => Some(Entry),
        // entry_e2e — zephyr-rust feature-workspace entries.
        (
            PlatformId::ZephyrNativeSim,
            Lang::Rust,
            Workload::Params | Workload::Qos | Workload::Lifecycle | Workload::Safety,
            Kind::Workspace,
        ) => Some(Entry),
        // workspace_features_e2e — native feature workspaces (Params is a
        // separate W4 file, deliberately excluded).
        (
            PlatformId::Linux,
            _,
            Workload::CustomMsg
            | Workload::Logging
            | Workload::Qos
            | Workload::Lifecycle
            | Workload::Safety
            | Workload::Remap,
            Kind::Workspace,
        ) => Some(WorkspaceFeatures),
        _ => None,
    }
}

// =============================================================================
// Realtime-dim matrix (RFC-0051 §6a, phase-329 W2)
// =============================================================================

/// The scheduler knob whose HONORING (or honest fallback) a sched-dim cell
/// asserts at boot. Each is a real-time tier attribute the `ws-realtime`
/// fixtures declare in `system_model.yaml`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SchedDim {
    /// `core_id` pin → `k_thread_cpu_pin` / `pthread_setaffinity` / … .
    CorePin,
    /// `deadline_us` → `k_thread_deadline_set` (Zephyr EDF).
    EdfDeadline,
    /// `preempt_threshold` → ThreadX `tx_thread_preemption_change`.
    PreemptThreshold,
    /// `time_slice_us` → ThreadX `tx_thread_time_slice_change`.
    TimeSlice,
    /// `budget_us`/`period_us` → NuttX `SCHED_SPORADIC`.
    SporadicBudget,
    /// tier priority → NuttX per-tier `SCHED_FIFO` priority.
    TierPriority,
    /// issue 0260 — `core` verified as PLACEMENT on a MULTI-CORE image: the
    /// tier is observed running on the cpu it declared.
    ///
    /// Distinct from [`SchedDim::CorePin`], which verifies that the kernel
    /// ACCEPTED the pin. On a uniprocessor image acceptance is true and says
    /// nothing — cpu 0 is the only cpu the tier could be on — so the two dims
    /// assert genuinely different claims and both are worth keeping.
    CorePinPlacement,
}

/// One realtime-dim matrix cell: a `(dim × platform × lang)` coordinate. Like
/// [`Cell`] this is NEUTRAL — the boot mechanism, markers, and assert shape are
/// the consumer's execution data (`tests/sched_dims_applied_e2e.rs`), not table
/// facts. Every cell boots a `ws-realtime-*` fixture (shared with
/// `realtime_tiers_e2e`).
#[derive(Copy, Clone, Debug)]
pub struct SchedCell {
    pub dim: SchedDim,
    pub platform: PlatformId,
    pub lang: Lang,
    pub tier: Tier,
}

const fn sched(dim: SchedDim, platform: PlatformId, lang: Lang, tier: Tier) -> SchedCell {
    SchedCell {
        dim,
        platform,
        lang,
        tier,
    }
}

/// The realtime-dim matrix (phase-329 W2 — replaces the 10 hand-written
/// `*_applied.rs` files, one `#[test]` each). Every dim a `ws-realtime` fixture
/// declares has a cell here; the consumer iterates them.
pub const SCHED_CELLS: &[SchedCell] = {
    use Lang::*;
    use PlatformId::*;
    use SchedDim::*;
    use Tier::Runtime;
    &[
        // core-pin — one per platform family that can pin (RFC-0052 fail-loud).
        sched(CorePin, ZephyrNativeSim, Rust, Runtime),
        sched(CorePin, NuttxArm, Rust, Runtime),
        sched(CorePin, ThreadxLinux, Rust, Runtime),
        sched(CorePin, FreertosMps2, Cpp, Runtime),
        sched(CorePin, Linux, Rust, Runtime),
        // issue 0260 — the ONLY multi-core cell: `core = 1` observed, not just
        // accepted. Zephyr/C on `qemu_cortex_a53/.../smp`.
        sched(CorePinPlacement, ZephyrNativeSim, C, Runtime),
        // Zephyr EDF deadline — all three language arms (W5.5 + W5.8).
        sched(EdfDeadline, ZephyrNativeSim, Rust, Runtime),
        sched(EdfDeadline, ZephyrNativeSim, Cpp, Runtime),
        sched(EdfDeadline, ZephyrNativeSim, C, Runtime),
        // NuttX sporadic + tier-priority.
        sched(SporadicBudget, NuttxArm, Cpp, Runtime),
        sched(TierPriority, NuttxArm, Rust, Runtime),
        // issue 0636 — the C/C++ arm of the same board. `nuttx_run_tiers.c`
        // took `&tiers[0]` as its session owner long after the Rust arm stopped,
        // and nothing noticed because no cell asserted tier priority on this
        // arm at all: the C arm applied priorities at `pthread_create` time and
        // printed nothing, so #579's "every declaring tier prints its marker"
        // was enforced in one language only. Both arms now route through
        // `nros_nuttx_apply_current_priority`, one implementation and one
        // marker, and this cell is what keeps them from diverging again.
        sched(TierPriority, NuttxArm, Cpp, Runtime),
        // issue 0636 — the FreeRTOS arm of the same rule. Added with the seam
        // that made it assertable: this kernel printed no tier-priority marker
        // in either language, so #579 was enforced on NuttX alone and a boot
        // task adopting NO priority at all had no cell that could see it. Both
        // languages, because one seam TU serves both entries.
        sched(TierPriority, FreertosMps2, Cpp, Runtime),
        sched(TierPriority, FreertosMps2, C, Runtime),
        // ThreadX preempt-threshold + time-slice.
        sched(PreemptThreshold, ThreadxLinux, Rust, Runtime),
        sched(TimeSlice, ThreadxLinux, Rust, Runtime),
    ]
};

/// Runtime sched-dim cells — what the consumer iterates.
pub fn sched_runtime_cells() -> impl Iterator<Item = &'static SchedCell> {
    SCHED_CELLS
        .iter()
        .filter(|c| matches!(c.tier, Tier::Runtime))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// phase-329 W1 gate G5 — the consumer-claim partition is sound.
    ///
    /// (1) Every Runtime cell of a FULLY-OWNED workload (`Multihost`,
    /// `RealtimeTiers`, workspace `Service`/`Action`) is claimed by a W1
    /// consumer — so adding such a cell to `CELLS` can never leave it un-run
    /// (the "adding a row adds no test" defect). (2) Every consumer still claims
    /// at least one Runtime cell — a tripwire against a workload/platform set
    /// silently vanishing from `CELLS`. Exec-arm totality over each claim is
    /// enforced at runtime (a claimed cell with no `exec_for` arm panics loudly
    /// in the consumer), since exec tables live in the per-test binaries.
    #[test]
    fn g5_w1_consumers_claim_their_owned_workloads() {
        for c in runtime_cells() {
            let fully_owned = matches!(c.workload, Workload::Multihost | Workload::RealtimeTiers)
                || (matches!(c.workload, Workload::Service | Workload::Action)
                    && matches!(c.kind, Kind::Workspace));
            if fully_owned {
                assert!(
                    w1_consumer_of(c).is_some(),
                    "G5: Runtime cell {c:?} is in a W1-owned workload but no consumer claims it \
                     — add it to `w1_consumer_of` and the consumer's `exec_for`"
                );
            }
        }
        for cons in [
            W1Consumer::Multihost,
            W1Consumer::Roundtrip,
            W1Consumer::RealtimeTiers,
            W1Consumer::Entry,
            W1Consumer::WorkspaceFeatures,
        ] {
            assert!(
                runtime_cells().any(|c| w1_consumer_of(c) == Some(cons)),
                "G5: consumer {cons:?} claims no Runtime cell — its workload/platform set \
                 vanished from CELLS (a consumer file would silently run nothing)"
            );
        }
    }

    /// phase-329 W2 — the realtime-dim table is populated and every declared
    /// `SchedDim` variant has at least one cell (a tripwire against a dim's
    /// cells vanishing from the table, the 0380 loss shape at the matrix layer).
    #[test]
    fn sched_dims_table_covers_every_dim() {
        assert!(!SCHED_CELLS.is_empty(), "the realtime-dim matrix is empty");
        for dim in [
            SchedDim::CorePin,
            SchedDim::EdfDeadline,
            SchedDim::PreemptThreshold,
            SchedDim::TimeSlice,
            SchedDim::SporadicBudget,
            SchedDim::TierPriority,
        ] {
            assert!(
                sched_runtime_cells().any(|c| c.dim == dim),
                "realtime-dim {dim:?} has no Runtime cell — its coverage vanished from SCHED_CELLS"
            );
        }
    }

    /// `Mixed` is a workspace-only axis value.
    #[test]
    fn mixed_lang_only_in_workspace_cells() {
        for c in CELLS {
            if matches!(c.lang, Lang::Mixed) {
                assert!(
                    matches!(c.kind, Kind::Workspace),
                    "Mixed cell outside Workspace: {c:?}"
                );
            }
        }
    }

    /// No duplicate coordinates — each (platform, lang, rmw, workload,
    /// kind) appears at most once.
    #[test]
    fn cells_unique() {
        let mut seen = std::collections::HashSet::new();
        for c in CELLS {
            let key = (
                c.platform.index(),
                c.lang as u8 as u16,
                c.rmw.index(),
                c.workload.port_offset(),
                c.kind as u8,
            );
            assert!(seen.insert(key), "duplicate cell: {c:?}");
        }
    }

    /// Every carve-out / build-only reason is non-empty (audit E5).
    #[test]
    fn gap_tiers_carry_reasons() {
        for c in CELLS {
            match c.tier {
                Tier::BuildOnly(r) | Tier::CarveOut(r) => {
                    assert!(!r.is_empty(), "empty reason: {c:?}")
                }
                Tier::Runtime => {}
            }
        }
    }
}
