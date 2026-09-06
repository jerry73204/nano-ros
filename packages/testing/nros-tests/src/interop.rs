//! Issue 0352 / phase-324 — THE interop & bridge test-intent list.
//!
//! [`crate::matrix::CELLS`] enumerates baked, self-contained cells. Interop and
//! bridge cells are a different shape: a nano side that is BUILT, plus an
//! ephemeral PEER (a stock ROS 2 node, an XRCE Agent, another nano bridge) and a
//! DIRECTION. `Cell` cannot carry a peer or a direction, so these cells live
//! here in the formulation their shape needs — an [`InteropCell`] wrapping the
//! nano [`Cell`] with `build` / `peer` / `dir` / `test`.
//!
//! The correspondence between what is TESTED (this list + `matrix::CELLS`), what
//! is BUILT (each cell's [`BuildChannel`], recipe named not invoked — the three
//! channels build DIFFERENTLY on purpose, issue 0352 non-goal: no unifier) and
//! what RUNS (each cell's `test`) is one [`Binding`] per cell, gated in
//! `tests/matrix_fixture_coverage.rs` (G1 coverage, G2 build-coord match, G3
//! tier, G4 peer-decl). A cell whose declared `(lang, rmw)` disagrees with the
//! fixture its test builds — the issue 0341 defect-2 drift class — is a gate
//! failure, not a silent pass.
//!
//! NOT modelled here: the docker per-edition harness (`ros_editions_e2e.rs`).
//! That is the ROS-edition axis (a per-run global, issue 0327), not a matrix
//! cell — it has no baked nano fixture in this list.

use crate::matrix::{Cell, Kind, PlatformId, Rmw, TestCell, Tier};

/// Which build channel produces an interop/bridge cell's NANO side.
///
/// The channels build differently on purpose; a channel only declares which
/// platform it can produce, so G2 can reject a cell pointed at a channel that
/// cannot build its coordinate.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BuildChannel {
    /// Native example / workspace-entry binaries (`just native build-fixtures`
    /// families). Host platform only.
    NativeFixtures,
    /// Zephyr workspace entry via the west leaves lane
    /// (`scripts/build/zephyr-fixture-leaves.sh`, driven by `just zephyr
    /// build-fixtures`).
    ZephyrWestLeaves,
}

impl BuildChannel {
    /// The `just` recipe that builds this channel's artifacts — NAMED, not
    /// invoked. Gated against the justfile by G2 the way `PlatformId::just_module`
    /// is by `just_module_names_a_real_module`.
    pub const fn build_recipe(self) -> &'static str {
        match self {
            BuildChannel::NativeFixtures => "just native build-fixtures",
            BuildChannel::ZephyrWestLeaves => "just zephyr build-fixtures",
        }
    }

    /// The `just` module the recipe lives under (what G2 checks exists).
    pub const fn just_module(self) -> &'static str {
        match self {
            BuildChannel::NativeFixtures => "native",
            BuildChannel::ZephyrWestLeaves => "zephyr",
        }
    }

    /// Can this channel build the given platform? The G2 coord check: a cell
    /// whose platform this channel cannot produce is a mis-declared binding.
    pub const fn builds_platform(self, p: PlatformId) -> bool {
        match self {
            BuildChannel::NativeFixtures => matches!(p, PlatformId::Linux),
            BuildChannel::ZephyrWestLeaves => matches!(p, PlatformId::ZephyrNativeSim),
        }
    }
}

/// The ephemeral peer a cell runs against. DECLARED, never built. A bridge names
/// BOTH endpoints.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Peer {
    /// A stock ROS 2 node of the run's edition (`NROS_ROS_EDITION`), speaking
    /// `rmw` (`rmw_zenoh_cpp` / `rmw_cyclonedds_cpp`).
    RosEdition(Rmw),
    /// nano XRCE client → micro-XRCE-DDS Agent → `rmw_fastrtps_cpp`.
    XrceAgent,
    /// nano declarative bridge: `ingress` rmw in → `egress` rmw out, then a ROS 2
    /// peer on the egress side.
    NanoBridge { ingress: Rmw, egress: Rmw },
}

impl Peer {
    /// True if the peer is internally consistent for `cell` (G4). For a
    /// single-RMW peer the rmw matches the cell; a bridge's `ingress` matches the
    /// cell's declared rmw (the nano side dials the ingress).
    pub fn consistent_with(self, cell: &Cell) -> bool {
        match self {
            Peer::RosEdition(rmw) => rmw == cell.rmw,
            // The XRCE Agent bridges nano-XRCE ⇄ fastrtps; the cell's rmw is Xrce.
            Peer::XrceAgent => matches!(cell.rmw, Rmw::Xrce),
            // A bridge cell's rmw is the INGRESS the nano side speaks.
            Peer::NanoBridge { ingress, egress } => ingress == cell.rmw && ingress != egress,
        }
    }
}

/// Which way data flows across the interop boundary.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Dir {
    /// nano-ros node → ROS 2 peer (nano is source / client).
    NanoToRos,
    /// ROS 2 peer → nano-ros node (nano is sink / server).
    RosToNano,
    /// Both directions in one test.
    BiDir,
}

/// One interop/bridge cell: the built nano side plus its peer, direction, build
/// channel and the test that runs it.
#[derive(Copy, Clone, Debug)]
pub struct InteropCell {
    /// Stable name, e.g. `"zephyr-qos-rust-zenoh"`. The `Binding` key.
    pub id: &'static str,
    /// The nano side. `cell.kind` is [`Kind::Interop`] or [`Kind::Bridge`];
    /// `cell.platform/lang/rmw/workload/tier` describe the built artifact.
    pub cell: Cell,
    /// How the nano side is built.
    pub build: BuildChannel,
    /// The ephemeral peer it runs against.
    pub peer: Peer,
    /// Data-flow direction.
    pub dir: Dir,
    /// The test binary (`cargo test --test <name>`) that runs this cell. A
    /// [`Tier::CarveOut`] cell that nothing runs carries [`NO_TEST`].
    pub test: &'static str,
}

/// `test` sentinel for a carved-out cell no test runs.
pub const NO_TEST: &str = "(carved-out — no runtime lane)";

impl TestCell for InteropCell {
    fn cell(&self) -> &Cell {
        &self.cell
    }
}

/// The correspondence row for one runnable test: which cell, built by which
/// recipe, run by which test. The row NAMES the recipes — it does not build or
/// run. This is the issue-0352 SSoT that ties BUILD and TEST together without
/// unifying either.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Binding {
    pub cell_id: &'static str,
    pub build_recipe: &'static str,
    pub test_recipe: &'static str,
}

impl InteropCell {
    /// The binding row for this cell.
    pub fn binding(&self) -> Binding {
        Binding {
            cell_id: self.id,
            build_recipe: self.build.build_recipe(),
            test_recipe: self.test,
        }
    }
}

const fn ic(
    id: &'static str,
    cell: Cell,
    build: BuildChannel,
    peer: Peer,
    dir: Dir,
    test: &'static str,
) -> InteropCell {
    InteropCell {
        id,
        cell,
        build,
        peer,
        dir,
        test,
    }
}

// Shorthand for the seed table.
use crate::matrix::{Lang::*, PlatformId::*, Rmw::*, Tier::*, Workload::*};
use BuildChannel::*;
use Dir::*;
use Kind::{Bridge, Interop};
use Peer::*;

/// Build a nano `Cell` for an interop/bridge row inline — the interop list is
/// the SSoT for these coordinates now, so it constructs its own cells rather
/// than referencing rows removed from `matrix::CELLS`.
const fn c(
    platform: PlatformId,
    lang: crate::matrix::Lang,
    rmw: Rmw,
    workload: crate::matrix::Workload,
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

/// THE interop & bridge cells (issue 0352 / phase-324). Moved out of
/// `matrix::CELLS` verbatim (the 6 native ROS-2 interop cells, the zephyr QoS
/// interop pair, the native lifecycle interop cell, the 2 declarative bridge
/// cells), now carrying their peer / direction / build channel / test.
#[rustfmt::skip]
pub const CELLS: &[InteropCell] = &[
    // ── Native nano ↔ stock ROS 2 (host), zenoh + cyclone ───────────────
    // tests/interop_e2e.rs — nano example bins vs `ros2 topic`/`ros2 service`.
    ic("native-pubsub-rust-zenoh-n2r",
       c(Linux, Rust, Zenoh, Pubsub, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), NanoToRos, "interop_e2e"),
    ic("native-service-rust-zenoh-r2n",
       c(Linux, Rust, Zenoh, Service, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), BiDir, "interop_e2e"),
    // phase-433 W3 — the cyclone half is C, not Rust. `interop_e2e`'s three
    // cyclone cases spawn `nano_cyclone_c_binary(...)`: `c_talker`,
    // `c_listener`, `c_service_server` out of `examples/native/c/`. These rows
    // said Rust, `scenario_coord` returned `Lang::Rust` unconditionally, and
    // the per-case tripwire compared the two — so the language axis was
    // inverted and agreed with itself. The vacated Rust/Cyclonedds shapes are
    // carved below.
    ic("native-pubsub-c-cyclone-n2r",
       c(Linux, C, Cyclonedds, Pubsub, Interop, Runtime),
       NativeFixtures, RosEdition(Cyclonedds), NanoToRos, "interop_e2e"),
    ic("native-service-c-cyclone-r2n",
       c(Linux, C, Cyclonedds, Service, Interop, Runtime),
       NativeFixtures, RosEdition(Cyclonedds), BiDir, "interop_e2e"),
    // The shapes the two rows above used to claim. Recorded rather than
    // dropped because the artifacts EXIST — `examples/native/rust/{talker,
    // listener,service-server,service-client}` all have `linux/rust/cyclonedds`
    // rows in `examples/fixtures.toml` — so the absence is a missing lane, not
    // a missing build, and nothing else in the tree would say so. Rust ↔
    // stock-ROS-2 over Cyclone is not wholly unproven: `native-graph-rust-
    // cyclone-r2n` runs that pairing for the Graph workload. Delivery is what
    // has never been run.
    ic("native-pubsub-rust-cyclone-n2r-CARVED",
       c(Linux, Rust, Cyclonedds, Pubsub, Interop,
         CarveOut("no Rust/Cyclonedds pubsub-interop lane; interop_e2e's cyclone \
                   pubsub cases run the C examples (c_talker/c_listener). The Rust \
                   cyclone talker/listener fixtures are built, so the lane is \
                   affordable — file one if wanted.")),
       NativeFixtures, RosEdition(Cyclonedds), NanoToRos, NO_TEST),
    ic("native-service-rust-cyclone-r2n-CARVED",
       c(Linux, Rust, Cyclonedds, Service, Interop,
         CarveOut("no Rust/Cyclonedds service-interop lane; interop_e2e's cyclone \
                   service case runs the C example (c_service_server). The Rust \
                   cyclone service-server/client fixtures are built, so the lane is \
                   affordable — file one if wanted.")),
       NativeFixtures, RosEdition(Cyclonedds), BiDir, NO_TEST),

    // ── phase-381 — READ the graph a stock ROS 2 node is in ──────────────
    // tests/graph_interop.rs. The only cell whose subject is DISCOVERY rather
    // than delivery, and the one that would have caught issue 0903: twelve
    // slots were produced, reachable from three languages, mutation-tested and
    // `check-api-parity`-clean while the feature did not work, because every
    // check tested our code against our own assumptions.
    ic("native-graph-rust-zenoh-r2n",
       c(Linux, Rust, Zenoh, Graph, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), RosToNano, "graph_interop"),
    // Cyclone's half. `graph.cpp` PUBLISHED `ros_discovery_info` since
    // phase-177.36 and only gained a reader in W5, which has never been run
    // against a live participant.
    ic("native-graph-rust-cyclone-r2n",
       c(Linux, Rust, Cyclonedds, Graph, Interop, Runtime),
       NativeFixtures, RosEdition(Cyclonedds), RosToNano, "graph_interop"),

    // ── phase-433 W6 — the ACTIONS family's live peer ────────────────────
    // tests/ros2_action_e2e.rs. Until this row the family had NO interop cell
    // at all: every action row in `matrix::CELLS` is nano-to-nano, and both
    // ends of such a pair share whatever convention `service.cpp`'s five CDR
    // adapters implement, so the one property the adapters exist to provide is
    // the one property those rows cannot observe (issue 0976). Actions are also
    // the family with the widest unexplained runtime spread — issue 0902
    // measured goals completing 20–90 % of the time on one build — which is a
    // shape only a live peer surfaces.
    //
    // BOTH directions, one coordinate. The adapters sit on both sides of the
    // service path: `strip_goal_id_len_at` / `strip_nested_cdr_at` fire only
    // when nano-ros WRITES a SendGoal/GetResult request, so the R2N cell (a
    // stock `ros2 action send_goal` into the nano server) does not reach them
    // and the N2R cell (the nano client into a stock server) is where they run.
    // `coords_for` collapses direction, so `assert_test_bound` in that file
    // names the coordinate once.
    ic("native-action-rust-cyclone-r2n",
       c(Linux, Rust, Cyclonedds, Action, Interop, Runtime),
       NativeFixtures, RosEdition(Cyclonedds), RosToNano, "ros2_action_e2e"),
    ic("native-action-rust-cyclone-n2r",
       c(Linux, Rust, Cyclonedds, Action, Interop, Runtime),
       NativeFixtures, RosEdition(Cyclonedds), NanoToRos, "ros2_action_e2e"),

    // ── Native nano XRCE ↔ Agent ↔ fastrtps ─────────────────────────────
    // tests/xrce_ros2_interop.rs.
    ic("native-pubsub-rust-xrce-n2r",
       c(Linux, Rust, Xrce, Pubsub, Interop, Runtime),
       NativeFixtures, XrceAgent, NanoToRos, "xrce_ros2_interop"),
    ic("native-service-rust-xrce-r2n",
       c(Linux, Rust, Xrce, Service, Interop, Runtime),
       NativeFixtures, XrceAgent, BiDir, "xrce_ros2_interop"),

    // ── Native nano lifecycle ↔ `ros2 lifecycle` ────────────────────────
    ic("native-lifecycle-rust-zenoh",
       c(Linux, Rust, Zenoh, Lifecycle, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), BiDir, "interop_e2e"),

    // ── Zephyr on-target QoS interop ────────────────────────────────────
    // Issue 0341 — the ONLY runtime test of this shape
    // (qos_zephyr_ros2_interop_e2e.rs) boots the RUST `ws-qos-rust` zephyr entry
    // over zenoh-pico → rmw_zenoh_cpp. The matrix used to declare Cpp/Cyclonedds,
    // which nothing ran (defect 2). Model reality here; carve the never-run shape.
    ic("zephyr-qos-rust-zenoh",
       c(ZephyrNativeSim, Rust, Zenoh, Qos, Interop, Runtime),
       ZephyrWestLeaves, RosEdition(Zenoh), BiDir, "qos_zephyr_ros2_interop_e2e"),
    ic("zephyr-qos-cpp-cyclone-CARVED",
       c(ZephyrNativeSim, Cpp, Cyclonedds, Qos, Interop,
         CarveOut("no zephyr Cpp/Cyclonedds QoS-interop lane; the QoS zephyr \
                   interop test runs Rust/Zenoh (zenoh-pico). File a lane if wanted.")),
       ZephyrWestLeaves, RosEdition(Cyclonedds), BiDir, NO_TEST),

    // ── Declarative cross-RMW bridges ───────────────────────────────────
    // The nano bridge is a `ws-bridge-*-rust` native_entry; a ROS 2 peer sits on
    // the egress side. cell.rmw = the INGRESS the nano side dials.
    ic("bridge-zenoh-to-cyclone",
       c(Linux, Rust, Zenoh, Pubsub, Bridge, Runtime),
       NativeFixtures, NanoBridge { ingress: Zenoh, egress: Cyclonedds }, NanoToRos,
       "declarative_bridge_zenoh_to_cyclonedds"),
    ic("bridge-zenoh-to-xrce",
       c(Linux, Rust, Zenoh, Pubsub, Bridge, Runtime),
       NativeFixtures, NanoBridge { ingress: Zenoh, egress: Xrce }, NanoToRos,
       "declarative_bridge_zenoh_to_xrce"),

    // ── Imperative (issue #53) zenoh→cyclone bridge — the G4 blind spot the
    //    binding closes (phase-329 W3). Same coordinate as the declarative
    //    sibling, distinct test. ──────────────────────────────────────────
    ic("bridge-zenoh-to-cyclone-imperative",
       c(Linux, Rust, Zenoh, Pubsub, Bridge, Runtime),
       NativeFixtures, NanoBridge { ingress: Zenoh, egress: Cyclonedds }, NanoToRos,
       "bridge_zenoh_to_cyclonedds"),

    // ── Native nano ↔ stock ROS 2, the previously-unbound live-peer lanes
    //    (phase-329 W3). Each file mixes host-only cases with a ROS-2-facing
    //    lane; the coordinate below is the interop lane's. ─────────────────
    // tests/qos_override_e2e.rs — a plan QoS override reaches the ADVERTISED
    // profile a stock rmw_zenoh_cpp peer reads (issue #52/0303/0306).
    ic("native-qos-override-rust-zenoh",
       c(Linux, Rust, Zenoh, Qos, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), NanoToRos, "qos_override_e2e"),
    // tests/params.rs — `ros2 param list/get/set` against a nano params entry.
    ic("native-params-rust-zenoh",
       c(Linux, Rust, Zenoh, Params, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), BiDir, "params"),
    // tests/rust_multi_node_per_node_graph.rs — a multi-node Rust entry shows
    // one graph node per launch component in `ros2 node list` (#104/phase-268).
    ic("native-multinode-rust-zenoh",
       c(Linux, Rust, Zenoh, EntryPubsub, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), NanoToRos, "rust_multi_node_per_node_graph"),
    // tests/cpp_multi_node_entry.rs — the C++ typed multi-node entry's pubsub +
    // per-node graph visibility against a stock ROS 2 peer (phase-257/268).
    ic("native-multinode-cpp-zenoh",
       c(Linux, Cpp, Zenoh, EntryPubsub, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), NanoToRos, "cpp_multi_node_entry"),

    // ── phase-433 W6 — a QoS STATUS EVENT fires against a live peer ──────
    // tests/qos_event_interop.rs. The four event slots (`publisher_event_init`,
    // `publisher_take_event`, `subscription_event_init`,
    // `subscription_take_event`) are `produced` and had never met a peer.
    //
    // They are the family least able to be tested in isolation: a QoS event's
    // input is a REMOTE entity's state. Four of the five kinds our ABI defines
    // are, in the zenoh shim, comparisons of our own clock against our own
    // timestamps — `LivelinessLost` and `OfferedDeadlineMissed` on the publisher
    // are literally self-observations. `LivelinessChanged` is the one whose
    // trigger is another process: the set of publishers holding an `@ros2_lv`
    // token matching the topic. That makes it the only INTEROP claim in the
    // family, because the token is written by `rmw_zenoh_cpp` and matched by our
    // wildcard, and the tree's only existing check of that wildcard matches it
    // against a keyexpr OUR OWN builder produced.
    //
    // Note what our ABI does NOT have: an incompatible-QoS kind. Upstream's
    // `RMW_EVENT_OFFERED_QOS_INCOMPATIBLE` / `REQUESTED_QOS_INCOMPATIBLE` have no
    // counterpart in `rmw_event_type_t`, so the cheapest event to provoke from a
    // stock peer is not one we can represent.
    ic("native-qos-event-rust-zenoh-r2n",
       c(Linux, Rust, Zenoh, QosEvents, Interop, Runtime),
       NativeFixtures, RosEdition(Zenoh), RosToNano, "qos_event_interop"),
    // Cyclone's half, carved rather than run — and the carve-out IS the W6
    // finding for that backend, measured from `vtable.cpp` rather than assumed.
    //
    // `subscription_event_init` and `publisher_event_init` are NULL there
    // (`kRegisterSubscriptionEvent` / `kRegisterPublisherEvent`, "deferred"
    // since phase 108). The other two slots, `subscription_take_event` and
    // `publisher_take_event`, ARE implemented and read real
    // `dds_get_*_status` counters — but nothing calls them: the Rust adapter
    // sets both to `None`, `nros-node` exposes no poll API, and a repo-wide
    // grep for `take_event` finds no consumer outside cyclonedds' own
    // `tests/status_events.cpp`. So a Cyclone application cannot observe a
    // status event by either half of the surface, and a live peer cannot change
    // that; the missing piece is a runtime poll path, not a lane.
    ic("native-qos-event-rust-cyclone-r2n-CARVED",
       c(Linux, Rust, Cyclonedds, QosEvents, Interop,
         CarveOut("cyclonedds cannot deliver a QoS status event to an application \
                   at all: both `*_event_init` slots are NULL, and the \
                   `*_take_event` pair it does implement has no caller in the \
                   tree outside its own C++ unit test. A live peer would change \
                   nothing — the gap is a runtime poll path. Issue 1164.")),
       NativeFixtures, RosEdition(Cyclonedds), RosToNano, NO_TEST),
];

/// Runtime interop/bridge cells only.
pub fn runtime_cells() -> impl Iterator<Item = &'static InteropCell> {
    CELLS
        .iter()
        .filter(|ic| matches!(ic.cell.tier, Tier::Runtime))
}

/// A nano coordinate `(platform, lang, rmw, workload)` — the granularity a test
/// binds to. Directions collapse (a cell may be exercised by an N2R and an R2N
/// case), so the binding is at coordinate level, not per-case.
pub type Coord = (u16, u16, u16, u16);

/// The distinct Runtime coordinates the given test's cells cover, per
/// `interop::CELLS`.
pub fn coords_for(test: &str) -> std::collections::BTreeSet<Coord> {
    CELLS
        .iter()
        .filter(|ic| ic.test == test && matches!(ic.cell.tier, Tier::Runtime))
        .map(|ic| {
            (
                ic.cell.platform.index(),
                ic.cell.lang.port_index(),
                ic.cell.rmw.index(),
                ic.cell.workload.port_offset(),
            )
        })
        .collect()
}

/// The interop cell with this id, if any.
pub fn by_id(id: &str) -> Option<&'static InteropCell> {
    CELLS.iter().find(|ic| ic.id == id)
}

/// Does `test` declare a Runtime cell at coordinate `(p, l, r, w)`? The runtime
/// per-case binding (issue 0352 / phase-324 W4.d): a test asserts, for each case
/// it actually runs, that the coordinate that case exercises is declared for it
/// in `interop::CELLS`. A case running a coordinate the SSoT does not list — the
/// 0341 defect-2 drift, seen from the test side — fails the running case.
pub fn test_covers(
    test: &str,
    p: PlatformId,
    l: crate::matrix::Lang,
    r: Rmw,
    w: crate::matrix::Workload,
) -> bool {
    coords_for(test).contains(&(p.index(), l.port_index(), r.index(), w.port_offset()))
}

/// Bind an interop test to `interop::CELLS`: assert the coordinates its `#[case]`s
/// exercise (`covered`, kept adjacent to the cases) are exactly those the list
/// declares for `test` (issue 0352 / phase-324 W4). Adding/retiring/mutating an
/// interop cell without tracking the test — or a test drifting from its cell's
/// declared coordinate (issue 0341 defect 2) — turns this RED.
///
/// Call it from a `#[test]` in the test binary; it needs no fixtures, so it runs
/// in tier 1 regardless of whether the runtime lane's ROS 2 / docker / QEMU
/// dependencies are present.
pub fn assert_test_bound(
    test: &str,
    covered: &[(
        PlatformId,
        crate::matrix::Lang,
        Rmw,
        crate::matrix::Workload,
    )],
) {
    let declared = coords_for(test);
    let actual: std::collections::BTreeSet<Coord> = covered
        .iter()
        .map(|(p, l, r, w)| (p.index(), l.port_index(), r.index(), w.port_offset()))
        .collect();
    assert_eq!(
        actual, declared,
        "interop test `{test}`: its #[case]s cover coordinates {actual:?}, but \
         interop::CELLS declares {declared:?} for this test — keep the cases and \
         interop::CELLS in sync (add/retire the row, or fix the drifted coordinate)"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every cell here is Interop or Bridge — the whole point of the split.
    #[test]
    fn only_interop_or_bridge_kinds() {
        for c in CELLS {
            assert!(
                matches!(c.cell.kind, Kind::Interop | Kind::Bridge),
                "non-interop/bridge cell in interop::CELLS: {c:?}"
            );
        }
    }

    /// Stable ids are unique — they are the `Binding` key.
    #[test]
    fn ids_unique() {
        let mut seen = std::collections::HashSet::new();
        for c in CELLS {
            assert!(seen.insert(c.id), "duplicate interop cell id: {}", c.id);
        }
    }

    /// Every carve-out reason is non-empty (audit E5, mirrored from matrix.rs).
    #[test]
    fn gap_tiers_carry_reasons() {
        for c in CELLS {
            if let Tier::CarveOut(r) | Tier::BuildOnly(r) = c.cell.tier {
                assert!(!r.is_empty(), "empty reason: {c:?}");
            }
        }
    }
}
