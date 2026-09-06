# Phase 433 W2 — preflight: which live-peer interop cells can produce a verdict

**Status (2026-09-06).** **Preflight only — nothing was run against a peer.** All
11 `interop::CELLS` test binaries COMPILE (measured, 11/11 `--no-run` green). The
open question W2 answers is which of the 17 `Tier::Runtime` cells can produce a
*verdict* once a ROS 2 Humble distrobox exists; this document is the input to
that. Four of `params.rs`'s eight tests can report **PASS on an unmet
precondition** and must be treated as producing no verdict until that is fixed
(see "The one real vacuous-pass"). Nothing here is a fix; every finding is
recorded, not applied.

Related: issue 1127 (eight of these binaries are named by no recipe),
`interop::CELLS` (`packages/testing/nros-tests/src/interop.rs`),
[docs/development/ros2-on-non-ubuntu.md](../development/ros2-on-non-ubuntu.md).

## What was MEASURED versus what was READ

**Measured** (commands re-runnable):

| claim | command |
| --- | --- |
| all 11 binaries compile | `cargo nextest run -p nros-tests --test <name> --no-run`, once per binary |
| the case list of each binary | `cargo nextest list -p nros-tests -E 'binary(x) or …'` |
| what `just test-all`'s own filter leaves | the same `list` intersected with the `justfile:1429` exclude expression |
| the zephyr QoS baked port matches the manifest | `port_of(ZephyrNativeSim, Rust, Qos)` = `7000 + 1*400 + 60 + 0` = **7460** = `west_zenoh_locator` in `examples/fixtures.toml:4643` |

**Read, not run**: every skip-guard analysis below, every fixture-row mapping,
and every statement about what a distrobox provides. This host has no ROS
(`/opt/ros` absent), so no runtime behaviour was observed — by design.

## Compile verdict: 11 / 11

Cold `nros-tests` build was ~3 min for the first binary, then 0.5–1 s each. No
errors, no warnings that `-D warnings` would catch. **No fix is needed and none
was applied.**

None of the 11 targets carries `required-features` — only `interop_e2e` even has
an explicit `[[test]]` stanza (`packages/testing/nros-tests/Cargo.toml:227`), and
it names no features. So the `check-required-features-reachable` class
(silently-never-built targets) does **not** apply to any of them; these are
plain auto-discovered integration tests and cargo builds them unconditionally.

## Correction to the phase premise: the sweep filter is not the blocker

The brief says none of the 17 cells is reached by any sweep. At the *filter*
level that is not what the tree says. `just test-all`'s exclude expression
(`justfile:1429`) leaves **9 of the 11 binaries and 25 of their 49 cases** in the
run. Measured:

| binary | in `just test-all`'s filter? | why |
| --- | --- | --- |
| `graph_interop` | **yes**, both cases | no test-group |
| `qos_override_e2e` | **yes**, all 3 | group `native-qos-discovery`, not excluded |
| `rust_multi_node_per_node_graph` | **yes**, both | no test-group |
| `cpp_multi_node_entry` | **yes**, all 4 | no test-group |
| `bridge_zenoh_to_cyclonedds` | **yes**, all 4 | no test-group |
| `declarative_bridge_zenoh_to_cyclonedds` | **yes**, all 3 | no test-group |
| `declarative_bridge_zenoh_to_xrce` | **yes**, both | no test-group |
| `qos_zephyr_ros2_interop_e2e` | **yes**, both | group `matrix-consumers-serial`, not excluded |
| `interop_e2e` | **only its 3 cyclone cases** | `binary(interop_e2e) and test(cyclone)` → `host-dds-ros2-interop`; the other 6 fall to `ros2-interop`, which IS excluded |
| `params` | **no** | group `ros2-interop`, excluded |
| `xrce_ros2_interop` | **no** | excluded by name |

So the accurate statement is narrower and worse in a different way: the cells are
inside tier 3's filter, tier 3 is the "pre-release, on demand" tier nobody runs
per task, and **on any host without ROS every one of them skips** — which is the
state this repo's CI has always been in. Six cells are additionally outside even
tier 3: `native-pubsub-rust-zenoh-n2r`, `native-service-rust-zenoh-r2n`,
`native-lifecycle-rust-zenoh` (all `interop_e2e`, non-cyclone), plus
`native-params-rust-zenoh`, `native-pubsub-rust-xrce-n2r` and
`native-service-rust-xrce-r2n`.

## The environment W2 has to assemble

| id | what | how | who needs it |
| --- | --- | --- | --- |
| **E1** | ROS 2 **Humble** under `/opt/ros/humble` | `scripts/dev/ros2-distrobox-setup.sh` (Ubuntu 22.04) | everything |
| **E2** | `rmw_zenoh_cpp` — and therefore the router `rmw_zenohd` | same script | 9 of 11 |
| **E3** | `rmw_cyclonedds_cpp` | same script | `graph_interop` cyclone case, `interop_e2e` cyclone cases, both cyclone bridges' ROS halves |
| **E4** | `rmw_fastrtps_cpp` + `example_interfaces` | same script | `xrce_ros2_interop` |
| **E5** | micro-XRCE-DDS Agent | `just xrce setup` **inside the box** (`NROS_HOME=$HOME/.nros-box`) | `xrce_ros2_interop`, `declarative_bridge_zenoh_to_xrce` |
| **E6** | vendored CycloneDDS provisioned | `just cyclonedds setup` in the box | every cyclone fixture ROW, so also the native fixture build itself |
| **E7** | Zephyr SDK + west | `just setup zephyr` in the box | `qos_zephyr_ros2_interop_e2e` only |

**The distro literal is not negotiable.** `DEFAULT_ROS_DISTRO = "humble"`
(`packages/testing/nros-tests/src/ros2.rs:13`) is what every probe interpolates:
`is_ros2_distro_available` sources `/opt/ros/{distro}/setup.bash`, and
`is_ros2_package_available` greps `ros2 pkg list` from the same prefix. A jazzy
box makes every probe return `false` and every case skip — the exact failure
`is_ros2_package_available`'s own doc-comment warns about. The setup script
already refuses a non-22.04 box, so this is a consistency note, not a gap.

**Two traps that will otherwise eat the session:**

1. **`source ./activate.sh` inside the box, not just `ros2-box-env.sh`.**
   `require_zenohd` resolves the router through
   `ros_zenohd_path_in` (`packages/testing/nros-tests/src/process.rs:1340`), whose
   only steps are `NROS_RMW_ZENOHD` → `AMENT_PREFIX_PATH` → `$ROS_DISTRO` under
   `/opt/ros`. `scripts/dev/ros2-box-env.sh` sets **neither** `AMENT_PREFIX_PATH`
   nor `ROS_DISTRO` (grepped: no match). `activate.sh:128` sources
   `/opt/ros/humble/setup.bash`, so it does. Without it, nine binaries skip on
   "zenohd not found" while a perfectly good router sits in `/opt/ros/humble`.
2. **Everything on the BOX tree** (`scripts/dev/ros2-box-sync.sh`, issue 0759) —
   fixture build included. A host-built fixture is a `GLIBC_2.xx not found` away
   from being useless here, and the fixture contract is leaf-relative so
   `CARGO_TARGET_DIR` redirection cannot paper over it.

## The build W2 needs, in order

```
# in the box, on the box tree, after ros2-box-sync.sh
. scripts/dev/ros2-box-env.sh && source ./activate.sh
just cyclonedds setup          # E6 — the native lane's cyclone rows need it
just xrce setup                # E5
just setup-cli                 # CLI FIRST: fixtures key on its source stamp
just build-test-fixtures lane=native
```

`lane=native` is the right lane and it is the *only* one needed for 10 of the 11.
Reasons, from `scripts/build/fixture-lane.sh`: `nros_lane_modules native` returns
exactly `native`, and `build-test-fixtures` additionally calls
`build-compile-check-fixtures` in its own body (`justfile:1585`), which is what
produces the `cpp_robot_entry` cmake fixture. Note that bare
`just native build-fixtures` does **not** do that second half — it is
`build-fixture-rust + build-fixture-extras + build-workspace-fixtures`
(`just/native.just:161`), so `cpp_multi_node_entry` would fail to resolve its
fixture. Use `build-test-fixtures lane=native`, not the module recipe.

The eleventh binary needs a **separate build channel**:

```
just zephyr build-fixtures     # only for qos_zephyr_ros2_interop_e2e
```

## Readiness table

Ordered most-ready first. "Verdict?" is the honest expectation once E1–E7 and the
build above are in place.

| # | binary | cells | compiles | extra beyond `lane=native` + Humble box | verdict? |
| --- | --- | --- | --- | --- | --- |
| 1 | `qos_override_e2e` | `native-qos-override-rust-zenoh` | ✅ | — | **yes**, and one half already gives one today |
| 2 | `rust_multi_node_per_node_graph` | `native-multinode-rust-zenoh` | ✅ | — | **yes** |
| 3 | `graph_interop` | `native-graph-rust-{zenoh,cyclone}-r2n` | ✅ | E3, E6 for the cyclone half | **yes** — the strongest-assertion pair in the set |
| 4 | `declarative_bridge_zenoh_to_xrce` | `bridge-zenoh-to-xrce` | ✅ | E5 | **yes**, and it needs no `ros2` CLI at all |
| 5 | `declarative_bridge_zenoh_to_cyclonedds` | `bridge-zenoh-to-cyclone` | ✅ | E3, E6 | **yes** — case 1 needs no ROS CLI, case 2 does |
| 6 | `bridge_zenoh_to_cyclonedds` | `bridge-zenoh-to-cyclone-imperative` | ✅ | E3, E6 | **yes** — 2 of 3 cases need no ROS CLI |
| 7 | `cpp_multi_node_entry` | `native-multinode-cpp-zenoh` | ✅ | the compile-check cmake fixture (see build note) | **yes**, but only 1 of its 4 tests is the cell |
| 8 | `interop_e2e` | 5 cells / 9 cases | ✅ | E3, E6 for the 3 cyclone cases | **yes** — the broadest single binary |
| 9 | `xrce_ros2_interop` | `native-{pubsub,service}-rust-xrce` | ✅ | E4, E5 | **probably** — 7 cases, and one carries a bus-hygiene skip that fires on a dirty domain |
| 10 | `params` | `native-params-rust-zenoh` | ✅ | — | **NO — 4 of 8 tests can PASS without exercising anything.** See below |
| 11 | `qos_zephyr_ros2_interop_e2e` | `zephyr-qos-rust-zenoh` | ✅ | **E7 + `just zephyr build-fixtures`** — a whole second build channel | **blocked on the zephyr lane**, not on ROS |

The 17th cell, `zephyr-qos-cpp-cyclone-CARVED`, is `Tier::CarveOut` with
`test = NO_TEST`. It is correctly excluded from W2 and needs nothing.

---

## Per-binary notes

### 1. `qos_override_e2e` — ready, and half of it already answers

Cells: `native-qos-override-rust-zenoh` (Linux, Rust, Zenoh, Qos).

Guards, in order:

- `the_bringup_declares_a_reliability_override_that_lowers` — **no guards at
  all.** Reads `examples/workspaces/features/src/demo_bringup/system.toml` and
  panics if the override is missing. This half produces a verdict on this host,
  today, with no fixtures. Its doc-comment states exactly why it exists: *"The
  runtime half below skips without ROS 2, so without this a silent edit to the
  bringup … would leave the whole file green-by-skipping."* That is the pattern
  the rest of this list should copy.
- `a_ros2_peer_sees_the_overridden_publisher_profile` — `zenohd_unique` rstest
  fixture (clean `or_skip`), then `require_zenohd()`, then `require_ros2()` (E1 +
  E2), then `build_native_workspace_rust_qos_entry()`.

Fixture: `[[workspace_fixture]] id = "workspace-features-rust-qos"`
(`examples/fixtures.toml:3653`), `examples/workspaces/features`, image
`native_rust_qos`. Built by `lane=native`.

Assertion strength: high. Per-endpoint (`one(&found[0], "PUBLISHER",
"reliable_talker")`), not a whole-report `contains`, and the comment records that
an earlier draft passed with the fix reverted. Nothing vacuous.

### 2. `rust_multi_node_per_node_graph` — ready

Cell: `native-multinode-rust-zenoh` (Linux, Rust, Zenoh, EntryPubsub).

Guards: `zenohd_unique` fixture → `require_zenohd()` → `is_ros2_available()` →
`is_rmw_zenoh_available()` → `build_native_workspace_rust_entry()`.

Fixture: `[[workspace_fixture]] id = "workspace-rust-native"`
(`examples/fixtures.toml:75`), `examples/workspaces/rust`, image `native`.

Assertion strength: fine — three assertions (`talker` present, `listener`
present, no bare `/node` line), all after a bounded poll. The poll returns
whatever it last saw on timeout, so a timeout lands on the assertions rather than
short-circuiting. Good.

### 3. `graph_interop` — the highest-value pair, and the cleanest

Cells: `native-graph-rust-zenoh-r2n`, `native-graph-rust-cyclone-r2n`.

Guards:

- zenoh case: `interop::assert_test_bound` (no fixtures, runs in tier 1) →
  `require_ros2()` → `fixtures::or_skip(ZenohRouter::start_unique())` →
  `Ros2Process::demo_nodes_cpp_talker` (needs `demo_nodes_cpp`, in
  `ros-humble-desktop`) → `fixtures::build_graph_probe()`.
- cyclone case: `require_ros2_cyclonedds()` (E3) → `unique_ros_domain_id()` →
  `Ros2DdsProcess::demo_nodes_cpp_talker_cyclonedds_with_domain` →
  `build_graph_probe_rmw(Cyclonedds)`.

Fixtures: two `[[fixture]]` rows on `packages/testing/nros-tests/bins/graph-probe`
(`examples/fixtures.toml:1395` zenoh / `:1407` cyclonedds), each
`no_default_features` + one `rmw-*` feature. Both in the native lane; the cyclone
one needs E6 to build.

Assertion strength: **the best in the set.** The probe exits non-zero unless it
sees the named peer, and the test asserts on the *status* as well as the marker,
plus `GRAPH_PROBE_ALL_SLOTS_OK`, plus the reverse direction via `ros2 node list`.
Its header explains why: issue 0903 was twelve slots that were mutation-tested
and parity-clean while the feature did not work. Recommend W2 runs this one
**first** — it is the cell most likely to find something.

### 4. `declarative_bridge_zenoh_to_xrce` — ready, and ROS-CLI-free

Cell: `bridge-zenoh-to-xrce`.

Guards: `zenohd_unique` fixture → `require_zenohd()` → `require_xrce_agent()`
(E5) → `build_native_workspace_rust_bridge_xrce_entry()` →
`build_int32_sink_rmw(Xrce)` → `build_native_talker_header()`.

Fixtures: `[[workspace_fixture]] id = "workspace-rust-native-bridge-xrce"`
(`examples/fixtures.toml:124`, rmw `xrce`), `[[fixture]]` int32-sink xrce
(`:1458`), `[[fixture]]` header-chatter-talker (`:1561`). All native lane.

Notable: **no `require_ros2*` anywhere.** The only ROS dependency is the router
binary itself. So this cell is verdict-capable on any host that can resolve
`rmw_zenohd` and has an XRCE Agent.

Assertion strength: fine. `received >= 2` on `INT32_LISTENER_LOG_PREFIX`, with
`wait_for_output_pattern` readiness gates on both the sink and the talker that
`.expect()` (i.e. fail, not skip) if the peer never speaks.

### 5. `declarative_bridge_zenoh_to_cyclonedds` — ready

Cell: `bridge-zenoh-to-cyclone`.

Two runtime cases:

- `declarative_zenoh_to_cyclonedds_bridge_to_nano_listener` —
  `require_zenohd()` → `build_native_workspace_rust_bridge_entry()` →
  `build_int32_sink_rmw(Cyclonedds)` → `or_skip(ZenohRouter::start_unique())` →
  `build_native_talker_header()`. **No ROS CLI needed.**
- `declarative_zenoh_to_cyclonedds_nested_header_to_ros2` —
  additionally `require_ros2_cyclonedds()` (E3) and a
  `Ros2DdsProcess::topic_echo_cyclonedds_with_domain` peer.

Fixtures: `[[workspace_fixture]] id = "workspace-rust-native-bridge"`
(`examples/fixtures.toml:109`, rmw `cyclonedds`), int32-sink cyclonedds
(`:1472`), header-chatter-talker (`:1561`). E6 is required for the workspace
entry — its own resolver doc says so: *"Gated on the cyclonedds submodule (the
entry build compiles vendored C++ Cyclone)."*

Assertion strength: fine (`received >= 2`; `count_pattern(&ros2_output, "sec:") >
0` for the nested case, which proves the nested `stamp` round-tripped rather than
just that bytes arrived).

### 6. `bridge_zenoh_to_cyclonedds` — ready

Cell: `bridge-zenoh-to-cyclone-imperative` (same coordinate as #5, distinct test —
that is the G4 blind spot the binding closes).

Three cases: base (`require_zenohd` + the fwd bin), path A (+ the cyclone C
listener), path B (+ `require_ros2_cyclonedds`). Only path B needs the ROS CLI.

Fixtures: `[[fixture]] dir = "…/bins/bridge-zenoh-to-cyclonedds-fwd"` rmw
`cyclonedds` (`examples/fixtures.toml:1326`), plus
`build_native_c_example_rmw("listener", "c_listener", Cyclonedds)` — the
`examples/native/c/listener` cyclone build. Both need E6.

Assertion strength: the base case asserts only the bridge's OWN `forwarded` log —
it proves `dds_write` was accepted, not that anything received. The file says so
in its own header and adds paths A and B for exactly that reason. Not vacuous,
but W2 should read the base case's green as the narrow claim it is.

### 7. `cpp_multi_node_entry` — ready, but only ¼ of it is the cell

Cell: `native-multinode-cpp-zenoh` (Linux, **Cpp**, Zenoh, EntryPubsub) — the one
non-Rust cell in the whole interop list.

Four tests, only one of which is the interop lane:

- `multi_node_workspace_cpp_typed_configures_and_builds` — inspects the prebuilt
  generated TU. No ROS, no router. A verdict today, given the fixture.
- `multi_node_workspace_cpp_typed_pubsub_e2e` — `zenohd_unique` +
  `require_zenohd()`. Two entry processes against a router; no ROS CLI.
- **`multi_node_workspace_cpp_per_node_graph_nodes`** — the cell.
  `require_zenohd()` → `is_ros2_available()` → `is_rmw_zenoh_available()`.
- `cases_bound_to_interop_cells` — the tripwire.

Fixture: `[[compile_check_fixture]] id = "cpp_robot_entry"`, builder
`cmake-configure`, `examples/templates/multi-node-workspace-cpp`, output
`src/robot_entry/robot_entry` (`examples/fixtures.toml:3358`). **This is the one
fixture in the set that is not a `[[fixture]]`/`[[workspace_fixture]]` row** — it
comes from `scripts/build/compile-check-fixtures.sh` via
`just build-compile-check-fixtures`, which `build-test-fixtures` calls but
`just native build-fixtures` does not.

Assertion strength: fine. Same three graph assertions as #2.

### 8. `interop_e2e` — the broadest, five cells in one binary

Cells: `native-pubsub-rust-zenoh-n2r`, `native-service-rust-zenoh-r2n`,
`native-pubsub-rust-cyclone-n2r`, `native-service-rust-cyclone-r2n`,
`native-lifecycle-rust-zenoh`. Nine rstest cases collapse onto those five
coordinates (`scenario_coord` maps every case to `(Linux, Rust, rmw, workload)`).

Guards: a per-case coordinate tripwire (`interop::test_covers`) that runs BEFORE
the environment gate — deliberately, so drift fails even when the runtime
dependency is absent — then `require_cell_env(scenario)`, which is
`require_ros2_cyclonedds()` for cyclone cases and `require_ros2()` for the rest.
Then `start_zenoh_router()` (skips on router failure) and per-scenario fixture
resolution.

Fixtures reached: `talker_binary`, `listener_binary`, `service_server_binary`,
`service_client_binary`, `lifecycle_node_binary` (native rust example rows),
`build_ros2_string_interop` (`examples/fixtures.toml:1313`), and
`build_native_c_example_rmw(…, Cyclonedds)` for the three cyclone cases (E6).
All native lane.

Peers: `ros2 topic echo` / `ros2 topic pub` / `ros2 service call` / `ros2 service`
server / `demo_nodes_cpp talker` / `ros2 lifecycle`. All in `ros-humble-desktop`
plus `example_interfaces` for `AddTwoInts`.

Assertion strength: good. Each case carries a `note` recording the fail-loud
intent (`"#133 fail-loud: … receiving 0 `data:` samples is a real rmw_zenoh
delivery failure, not timing"`), and delivery goes through `assert_delivery`.

Caveat for W2: `run_ros2` / `poll_ros2_until` pass `--no-daemon` so the CLI uses
the test's own zenoh session. Do not "fix" a slow lifecycle case by dropping that.

### 9. `xrce_ros2_interop` — ready, with two things to know

Cells: `native-pubsub-rust-xrce-n2r`, `native-service-rust-xrce-r2n`. Seven
runtime cases (pubsub both ways, service both ways, action both ways, concurrent
action).

Guards, uniform: `require_xrce_agent()` (E5, `just xrce setup`) then
`require_ros2_dds()` (E1 + E4). The seven binaries come from rstest fixtures
(`xrce_talker_binary`, `xrce_listener_binary`, `xrce_service_{server,client}_binary`,
`xrce_action_{server,client}_binary`, `xrce_action_server_concurrent_binary`) —
these are `.expect()` wrappers, so an unbuilt fixture is a hard failure via the
resolver's own tier-aware skip, not a silent pass.

Two things W2 should not be surprised by:

- **A bus-hygiene skip that is not a bug.** `test_xrce_service_ros2_client`
  probes `service_present_on_domain(…, "/add_two_ints")` before starting anything
  and skips if a foreign server already answers. That is issue 0741's fix — a
  poisoned bus cannot answer the question, and calling it an interop regression
  produced four retracted diagnoses. The `dds_isolation` loopback profile (issue
  1009) should make this rare inside a box, but a stray host-side
  `add_two_ints_server` will still trip it.
- **Two stale comments, neither load-bearing.** The module header still says
  *"These tests are diagnostic/informational — they report interop status but do
  not hard-fail the test suite"*; that has not been true since the vacuous
  `test_ros2_dds_detection` was deleted — every case now ends in a real
  `assert!`. And `test_ros2_action_xrce_client`'s skip message blames
  `action_tutorials_py`, while the peer is an inline rclpy script importing only
  `example_interfaces.action.Fibonacci`. Recording both; not fixing them here.

### 10. `params` — **the one real vacuous-pass, four tests deep**

Cell: `native-params-rust-zenoh`.

Eight tests. Four of them — `test_ros2_param_list`, `test_ros2_param_get`,
`test_ros2_param_set`, `test_ros2_param_describe` — contain this, at
`params.rs:189`, `:232`, `:276` and `:323`:

```rust
    if !require_node_discoverable(&locator) {
        return;
    }
```

and `require_node_discoverable` (`params.rs:157`) is:

```rust
fn require_node_discoverable(locator: &str) -> bool {
    for attempt in 1..=3 {
        if let Ok(output) = nros_tests::ros2::ros2_node_list(locator, "humble")
            && output.contains("/demo/talker")
        { return true; }
        …
    }
    eprintln!(
        "Skipping test: nros node /demo/talker not discoverable via ros2 \
         (may be zenohd version mismatch or rmw_zenoh configuration issue)"
    );
    false
}
```

A bare `return` from a `#[test]` body is a **PASS**. So on the exact host these
tests were written for — ROS present, router present, fixture built, and the
nano-ros node simply not showing up in `ros2 node list` — all four report green
while asserting nothing. The `eprintln!` says "Skipping test"; nextest does not
agree, and under `--failure-output never` the line is not even printed.

This is precisely the class CLAUDE.md names, one notch subtler than
`check-no-vacuous-tests` can see: that gate keys on a body whose *only* effects
are prints, and these bodies have plenty of other effects on the path where the
guard passes. **The obvious one-line fix is `nros_tests::skip!(…)` in place of
each `return;`** — a `skip!` panic is what the `test-all` junit rewrite turns into
a real skip. It is not applied here (this preflight changes no test file).

Until that lands, **W2 must treat a green `params` run as no evidence** and read
the four tests' stdout to confirm the discovery probe actually succeeded. The
other four tests (`test_talker_uses_default_param`,
`test_talker_param_declaration`, `test_param_integer_type`,
`test_ros2_param_set_reconfigures_live_read`) do not have this shape; the last of
those uses a proper `nros_tests::skip!` on the same discovery failure
(`params.rs:514`), which is the correct spelling sitting three functions away
from the wrong one.

Fixtures: `[[fixture]] dir = "…/bins/param-chatter-talker"`
(`examples/fixtures.toml:1384`), `[[fixture]]` int32-sink zenoh (`:1419`),
`[[workspace_fixture]] id = "workspace-features-rust-params"` (`:3664`). All
native lane.

Secondary note: `require_node_discoverable` and the `ros2_param_*` helpers pass
the string literal `"humble"` rather than `DEFAULT_ROS_DISTRO`. Harmless while
they are the same value, and exactly the drift
`is_ros2_package_available`'s doc-comment describes.

### 11. `qos_zephyr_ros2_interop_e2e` — blocked on a second build channel

Cell: `zephyr-qos-rust-zenoh` — the only non-Linux Runtime cell in the list, and
the only one whose `BuildChannel` is `ZephyrWestLeaves` rather than
`NativeFixtures`.

Guards: `require_ros2()` (E1 + E2) → `build_zephyr_workspace_rust_qos_entry()`
(skips with the west error) → `ZenohRouter::start_on("127.0.0.1", 7460)` (skips if
the port is busy) → `ZephyrProcess::start(&entry, NativeSim)` — which
**panics**, not skips, if the image will not boot.

Fixture: `[[workspace_fixture]] id = "workspace-zephyr-rs-qos"`
(`examples/fixtures.toml:4632`), west build name
`build-ws-rs-qos-entry-zenoh`, resolved from
`<zephyr-build-root>/build-ws-rs-qos-entry-zenoh/zephyr/zephyr.exe`. **Not in
`lane=native`** — `nros_lane_modules native` returns only `native`. It needs
`just zephyr build-fixtures`, i.e. E7 plus a west workspace in the box.

The baked router port is not a magic number and it checks out: the test computes
`port_of(ZephyrNativeSim, Rust, Qos)` = `7000 + 1*400 + 60 + 0` = **7460**, which
is byte-for-byte the manifest's `west_zenoh_locator = "tcp/127.0.0.1:7460"`.
Because the port is fixed rather than ephemeral, this test cannot run concurrently
with the `entry_e2e` `zephyr_rust_qos` cell — hence its
`matrix-consumers-serial` test-group (`.config/nextest.toml:439`).

Assertion strength: fine — a bounded 45 s poll of `ros2 topic echo --once`, then
`assert!(delivered, …)`.

**Recommendation: schedule this one last, or split it out of W2.** It shares
nothing with the other ten but the ROS install; a Zephyr west workspace in a
distrobox is its own project.

---

## Suggested W2 order

1. **Build once**: box tree → `cyclonedds setup`, `xrce setup`, `setup-cli`,
   `build-test-fixtures lane=native`. Everything below reuses it.
2. **`graph_interop`** — strongest assertions, no exotic fixture, and the cell
   most likely to surface a real defect (its whole reason for existing).
3. **`qos_override_e2e`, `rust_multi_node_per_node_graph`, `cpp_multi_node_entry`** —
   single-cell, single-assertion-family, fast to triage.
4. **`interop_e2e`** — five cells at once; expect the cyclone cases to need a
   clean domain (`unique_ros_domain_id` handles it, `dds_isolation` pins the bus).
5. **The three bridges** — two of the three have cases that need no ROS CLI, so a
   partial green here still says something.
6. **`xrce_ros2_interop`** — after `just xrce setup` is confirmed in the box.
7. **`params`** — run it, but read stdout; a green is not a verdict until the four
   bare `return`s become `skip!`s.
8. **`qos_zephyr_ros2_interop_e2e`** — separate session, separate build channel.

## Things that cannot work by construction

Nothing in the set is structurally impossible. Specifically checked and clear:

- No target is behind an unreachable `required-features` — none of the 11 has any.
- Every fixture a test names has a manifest row (or, for `cpp_robot_entry`, a
  `[[compile_check_fixture]]` row). No test resolves a binary nothing builds.
- Every `interop::CELLS` row with `test != NO_TEST` names a binary that exists and
  compiles, and every one of those binaries carries its
  `interop::assert_test_bound` tripwire — so the coordinate correspondence is
  already gated and already green (all 11 compiled, and the tripwire tests need no
  fixtures).

The one thing that is *effectively* broken is `params`'s four bare returns, and
that is a source fix, not a provisioning problem.
