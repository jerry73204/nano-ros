# Phase 304 — Complete the ROS edition axis

**Status (2026-07-25).** PARTIAL. W3 landed (enum extended); W2 core + W4
started; **W1 not started** — the biggest functional gap, and it needs the
capture run. Two knobs still address one axis: codegen selection (`--ros-edition`)
and runtime selection (the cargo feature) are disconnected and can silently
disagree. Only `humble` is exercised. Note this phase coordinates
[phase-303](phase-303-xcdr2-interop.md), whose premise was REFUTED after this
status was written — re-read 303's correction before planning further work here.

Implements **[RFC-0056](../design/0056-ros-edition-axis.md)** (the ROS-edition
axis / per-distro interop profile). Coordinates the two field-phases —
**[phase-41](archived/phase-41-iron-type-hash-support.md)** (RIHS01 type hash) and
**[phase-303](phase-303-xcdr2-interop.md)** / **[RFC-0055](../design/0055-wire-encoding-xcdr2-extensibility.md)**
(wire encoding) — into a finished axis, adds the **unified selection** and the
**multi-distro test method**, and extends the enum beyond `humble`/`iron`. Roots:
issue **#0267** (the encoding half) and the discovery-rejection risk phase-41
flagged (the type-hash half). Test lanes plug into
**[RFC-0051](../design/0051-test-matrix-architecture.md)** (the edition becomes a
matrix axis).

## Status (2026-07-25)

**W3 LANDED** (extend the enum). **W2 core + W4 started** (this commit):
`[system].ros_edition` is declared + resolved + typo-guarded; `nros_tests::ros2`
is distro-parametric; the Tier-A capture script exists. **W1 not started** (the
biggest functional gap; needs the capture run). The axis is **partially
built**: `RosEdition {Humble, Iron}` +
`--ros-edition` CLI arg + `ros-humble`/`ros-iron` cargo features + a runtime
keyexpr branch exist, but (a) `RosEdition::type_hash()` returns a **placeholder**
for Iron (`RIHS01_<64×0>`, not computed), (b) codegen selection (the CLI arg)
and runtime selection (the cargo feature) are **disconnected** — two knobs for
one axis that can silently disagree, and (c) only `humble` is exercised.

## The problem: the CI/dev host installs only Humble

`/opt/ros/` has **humble only**; `nros_tests::ros2` hard-codes
`/opt/ros/humble/setup.bash` in its availability checks (though
`ros2_env_setup(distro)` is already distro-parameterized), and the pinned
`rmw_zenoh` (1.7.2) is Humble-compat. So "test on Iron/Jazzy" needs a way to
reach those distros without a host-wide install. Two tiers:

### Tier A — offline, no distro install (codegen correctness)

Everything a distro changes in the GENERATED artifacts is capturable **once**
into committed fixtures and tested in-repo with no ROS runtime:

- **RIHS01 reference hashes** — `ros2 interface hash <type>` on the distro
  yields the `RIHS01_<sha256>` for each type. Capture a small set
  (`std_msgs/Int32`, a nested-struct type, a service), commit as fixtures, and
  assert the Rust REP-2011 computation (phase-41 W1) reproduces them
  byte-for-byte. This is exactly the "collect reference hashes from Iron/Jazzy"
  TODO phase-41 §41.1 left open. **No live peer needed.**
- **Per-distro `rosidl_adapter` IDL** — the `nros-msg-to-idl` parity contract
  (`tests/parity.rs`) is Humble-captured today; capture the Jazzy/Iron
  `rosidl_adapter` output per fixture and assert the emitter matches the
  SELECTED edition's reference (extensibility annotation, if any, is a per-distro
  fixture — this is where the phase-303 W1 finding gets pinned down per distro).
- **Per-distro `.msg` deltas** — the interface set differs; capture the target
  distro's `.msg` for the fixtures.

Capture source: a throwaway **`osrf/ros:<distro>`** container (`ros:iron-ros-base`,
`ros:jazzy-ros-base`) run once by a `scripts/ros/capture-edition-fixtures.sh`
helper — the container is a capture tool, not a test dependency.

### Tier B — live wire interop (needs the distro at test time)

Actual discovery + on-wire decode against a real ROS 2 node of the target
distro. Two options; the matrix picks per-lane:

- **B1 — container peer (recommended, no host install).** The interop test
  spins an `osrf/ros:<distro>` container as the ROS 2 peer (`ros2 topic echo` /
  a small node) and nano-ros (host) talks to it over the network (zenoh router
  or DDS). Needs the distro's matching `rmw_zenoh` (version per distro) inside
  the container. Isolated, CI-friendly, coexists with the humble host.
- **B2 — multi-`/opt/ros` install.** apt-install `ros-iron-*` / `ros-jazzy-*`
  alongside humble (they coexist under `/opt/ros/`), generalize the
  `nros_tests::ros2` hard-coded `/opt/ros/humble` availability checks to the
  requested distro, and source the right overlay. Simplest on a dev box; a
  provisioning change for CI.

**Skip discipline (RFC-0051 / the fail-loud rule):** an edition lane whose
distro (container image or `/opt/ros/<distro>`) is absent must
`nros_tests::skip!` — never a bare pass. Only `humble` runs unconditionally.

## Work items

### W1 — real RIHS01 computation (drives phase-41)

Replace `RosEdition::type_hash()`'s Iron placeholder with the REP-2011 RIHS01
(`docs/research/rep-2011-type-hash.md`): add `sha2` to `rosidl-codegen`,
implement the canonical type-description form + SHA-256, gate the real hash on
`iron`+ (keep the Humble `TypeHashNotSupported`).

**W1 engine LANDED (2026-07-25):** `rosidl_codegen::rihs` — the pure engine.
`FieldTypeDesc`/`FieldDesc`/`IndividualTypeDescription`/`TypeDescription` (the
REP-2011 type-id enum + array/sequence offsets), `to_hashable_json` (libyaml-flow
canonical form, fixed key order, referenced-types sorted / fields kept), and
`rihs01` (SHA-256 → `RIHS01_<64hex>`). Verified: the canonical JSON matches the
research doc's documented `std_msgs/msg/Int32` reference BYTE-FOR-BYTE; the
engine's Int32 hash is snapshot-locked
(`RIHS01_22ff2de7…f99b6`) as a regression guard. rihs.rs is clippy-clean.

**W1b (a) LANDED (2026-07-25) — AST → TypeDescription:** `field_type_desc`
(primitive/string/bounded-string → scalar id; array/sequence/bounded-sequence →
element base + `+48/+96/+144` offset with capacity/string_capacity; namespaced →
NESTED_TYPE + `pkg/msg/Name`), `message_to_individual` (fields only, source
order), and `build_type_description` (DAG closure over nested refs via a
caller-supplied `resolve` callback — de-duped, loud error on an unresolvable
ref). Tested: the primitive/array/`string<=20[]`→161/bare-bounded-string→21/
nested mappings + a two-level DAG closure + the unresolvable-ref error. Bounded
strings in collections + wstrings are best-effort pending the (b) Jazzy
confirmation.

**W1b (b) CONFIRMED against LIVE Jazzy (2026-07-25):** ran the capture against a
`ros:jazzy-ros-base` container and read the rosidl type-description `.json`s. The
loop FOUND A REAL BUG — the hashable form is NOT compact (the research doc
guessed wrong): `calculate_type_hash` uses
`json.dumps(separators=(', ', ': '), ensure_ascii=True, sort_keys=False)` with
`default_value` stripped. Fixed `to_hashable_json` (spaced separators + ASCII
escaping); the engine now reproduces the REAL Jazzy hashes byte-for-byte for
`std_msgs/msg/Int32` (`RIHS01_b6578ded…`), `std_msgs/msg/Header` (nested Time +
string, `RIHS01_f49fb3ae…`), and `builtin_interfaces/msg/Time`
(`RIHS01_b106235e…`) — all locked as unit-test assertions. Reference hashes
committed in `fixtures/ros-editions/jazzy/hashes.txt`; the research doc §4 is
corrected. The capture script now reads the `.json` type descriptions (there is
no `ros2 interface hash` subcommand — the doc's claim was also wrong).

**W1b (c) LANDED (2026-07-25) — codegen wiring:** `generate_package`
(`rosidl-bindgen`) now computes the per-message `TYPE_HASH` via
`compute_msg_type_hash` — Humble keeps the `TypeHashNotSupported` placeholder
(`edition.uses_type_hash()` gate); Iron/Jazzy/Rolling build the REP-2011
`TypeDescription` DAG and emit the real `RIHS01_<hash>`. The emission fns
(`generate_nros_message_package` / `generate_nros_inline_message`) now take a
`type_hash: &str` (hash decided by the caller) instead of an `edition`.
Nested-type closure: `generate_package` resolves **same-package** nested types
itself from `package.share_dir`; **cross-package** types come from a
caller-supplied `MsgResolver` (`nros sync` + `cargo-nano-ros` build one over the
ament interface index; self-contained/Humble paths pass `no_cross_pkg_resolver`).
An unresolvable nested type is a **HARD error** — never a wrong or placeholder
hash on the wire. Codegen-level assertions in `generator.rs` pin the real Jazzy
Int32 (`b6578ded…`, flat) and Header (`f49fb3ae…`, nested Time via resolver)
hashes byte-for-byte, plus the Humble-placeholder and fail-loud paths.

- *Accept (W1b):* ✅ the engine reproduces the Tier-A captured reference hashes
  for the fixture set byte-for-byte (rihs unit tests + the codegen-level
  `generator.rs` tests); `humble` unchanged (placeholder).

**W1b (c) note:** the runtime keyexpr/liveliness path was ALREADY baked — it
reads `M::TYPE_HASH` (the codegen constant, `node.rs:266/297` →
`TopicInfo::new`), NOT `RosEdition::type_hash()`. So a Rust pub/sub build carries
the real per-type hash the moment codegen runs with an Iron+ edition. The
remaining gap is service/action — see **W1c**.

### W1c — service / action `_Event` synthesis (REP-2011) — **LANDED (2026-07-25)**

Service + action `TYPE_HASH`/`SERVICE_HASH`/`ACTION_HASH` now carry the REAL
RIHS01 on Iron+ (placeholder on Humble). rcl hashes the WHOLE service/action
type-description DAG, including synthesized `_Request`/`_Response`/`_Event`
members — the `rosidl_codegen::rihs` engine now synthesizes them.

**Engine:** `build_service_type_description` (3-member top + `_Event`),
`service_member_type_description` (standalone `_Request`/`_Response`),
`build_action_type_description` + `action_type_hashes` (six-member top + two
nested service triads → nine distinct hashes). Fixed built-ins
(`service_msgs/ServiceEventInfo`, `builtin_interfaces/Time`,
`unique_identifier_msgs/UUID`) are EMBEDDED constants — no ament dependency. An
empty message gets the rosidl `structure_needs_at_least_one_member` placeholder
(handled in `message_to_individual`).

**Codegen wiring:** the service template split its single hash into
`request_type_hash`/`response_type_hash`/`service_hash`; the action template into
nine named slots. `generate_package` computes them via
`compute_service_type_hashes` / `compute_action_type_hashes` (Humble → placeholder;
unresolvable nested user type → HARD error).

**Verified byte-for-byte vs live Jazzy:** all 5 `std_srvs/srv/SetBool` hashes +
`ServiceEventInfo`; all 9 `tf2_msgs/action/LookupTransform` hashes (rihs unit
tests) + the empty-`_Feedback` placeholder; plus codegen-level tests
(`jazzy_service_emits_real_rihs01_hashes`, `jazzy_action_emits_nine_real_rihs01_hashes`).
Recipe + golden values: [`docs/research/rep-2011-type-hash.md`](../research/rep-2011-type-hash.md)
§3a/§3b + `fixtures/ros-editions/jazzy/srv-hashes.txt`.

Original design notes (retained):

Key facts the engine must encode:
- **Service top-level** = 3 `NESTED_TYPE` fields (`request_message`,
  `response_message`, `event_message`) in source order.
- **`_Event`** = `{ info: NESTED service_msgs/ServiceEventInfo; request:
  <Srv>_Request[<=1] (id 97); response: <Srv>_Response[<=1] (id 97) }`.
- **`service_msgs/msg/ServiceEventInfo`** is a FIXED built-in — embed its
  canonical ITD as a codegen constant (`event_type` u8, `stamp` Time, `client_gid`
  **uint8[16] id 51** — `.msg` says `char[16]` but rosidl maps `char`→uint8,
  `sequence_number` i64) rather than depending on the `service_msgs` package.
- **Action top-level** = 6 `NESTED_TYPE` fields (`goal`, `result`, `feedback`,
  `send_goal_service`, `get_result_service`, `feedback_message`); the two nested
  services each reuse the §3a `_Event` synthesis; `goal_id` uses
  `unique_identifier_msgs/msg/UUID`.

- *Accept (W1c):* the engine reproduces the committed SetBool + LookupTransform
  hashes byte-for-byte (codegen-level tests), and `generate_nros_service_package`
  / `_action_package` emit the real `RIHS01_…` on Iron+ (placeholder on Humble).

### W2 — unify edition selection (`[system].ros_edition`, RFC-0056 open-Q1)

Declare the edition ONCE in `system.toml [system].ros_edition` (default
`humble`) and lower it — like RMW (RFC-0031) — to (a) the codegen
`--ros-edition`, and (b) the `ros-<edition>` cargo feature on the board/umbrella.
(Leg (c) `generated/<edition>/` was found UNNECESSARY — see the W2b note below.)
Kills the codegen↔runtime disconnect (baked type_hash must match the runtime
keyexpr tail).

- *Accept:* a `[system].ros_edition = "iron"` workspace bakes the Iron type_hash
  AND builds with `ros-iron` — no hand-set feature; a mismatch is impossible by
  construction. A missing/`humble` value is byte-identical to today.

**W2 core LANDED (2026-07-25):** `SystemHeader.ros_edition: Option<String>` +
`SystemHeader::ros_edition() -> Result<RosEdition>` (absent ⇒ humble; unknown ⇒
HARD error — typo guard, never a silent fallback). `nros codegen-system`
resolves + validates + records it at bake (a bad `[system].ros_edition` fails
loudly). Unit-tested (`ros_edition_resolves_with_humble_default_and_typo_guard`).

**W2b LANDED (2026-07-25) — the lowering.** SSoT `RosEdition::cargo_feature()`
(`ros-<edition>`, the twin `ResolvedRmw::cargo_feature` lacked). Threaded to:

- **(a) codegen `--ros-edition` default** — `nros sync --ros-edition` is now
  `Option`; when omitted it auto-lowers `[system].ros_edition` from a
  `system.toml` at the workspace root (`resolve_sync_edition`), else humble. The
  CMake C/C++ path: `nros_generate_interfaces` defaults `ROS_EDITION` from the
  workspace `NANO_ROS_ROS_EDITION`, and `nano_ros_generate_interfaces` now
  accepts+forwards `ROS_EDITION`.
- **(b) `ros-<edition>` cargo feature** — `nano_ros_workspace(EDITION …)` sets
  `NANO_ROS_ROS_EDITION`; the runtime umbrella (`NanoRosRuntimeCrate.cmake`) now
  emits `ros-${edition}` (was hardcoded `ros-humble`) into `_cpp_features`,
  exactly mirroring the RMW `_backend_feat`. The Rust scaffold (`nros new
  --ros-edition`) emits the matching `ros-<edition>` on the `nros` dep. Since the
  codegen hash (a) and the keyexpr feature (b) both derive from the ONE edition
  value, a mismatch is impossible by construction.
- **bake C-define parity** — `codegen-system` emits `#define
  NROS_SYSTEM_ROS_EDITION "<e>"` + `…_<E>` for a non-humble edition (humble/absent
  emits nothing → byte-identical).

Verified: `cargo_feature` SSoT test; `system_config_h_emits_ros_edition_only_when_non_humble`
(humble byte-identical); all 4 edited CMake files parse; codegen-system (20) +
scaffold + rosidl-codegen suites green.

**W2b leg (c) `generated/<edition>/` interface dir — UNNECESSARY (dropped,
2026-07-25).** Re-analysis: a per-edition subdir in the WORKSPACE codegen output
is not needed. As in standard ROS (one sourced distro → one `install/`, never
mixed), a nano-ros workspace declares exactly ONE `[system].ros_edition`; every
generated crate targets it. Switching edition regenerates IN PLACE, correctly,
because:
- the edition is baked into generated *content* (`TYPE_HASH`), and
  `write_if_changed` is content-based → a changed edition → different hash →
  rewrite;
- the CMake codegen args file embeds `"ros_edition"` and the codegen
  `add_custom_command` has `DEPENDS … "${_args_file}"` → an edition change
  re-triggers codegen.

So the flat `generated/<pkg>` layout is already coherent, and (a)+(b) guarantee
the codegen↔runtime match. A per-edition subdir would only matter if two editions
had to coexist in ONE workspace — which never happens, exactly as upstream ROS
bindings are single-edition.

NOTE (separate, pre-existing): `packages/interfaces/*/generated/{humble,iron}/`
are committed per-edition dirs from an earlier "edition-aware binding" attempt.
They are ORPHANED — the parent crates carry no path dep on them, the bundled
interface resolver returns the parent dir (not a per-edition subdir), and the
lone `iron/` copy holds STALE `RIHS01_0000…` placeholder hashes (pre-W1). That is
the vendored-prebuilt-bindings idea, distinct from W2b; tracked as cleanup
(regenerate via the W1c engine or drop to humble-only + regenerate-on-demand).

**⇒ W2b is COMPLETE** — legs (a)+(b)+bake are the whole lowering; (c) was never
required.

### W3 — extend the enum: `jazzy` / `rolling` — **LANDED (2026-07-25)**

`RosEdition` gained `Jazzy`/`Rolling` + a single `RosEdition::parse` /
`as_str` / `uses_type_hash` API (every CLI parse site — `nros sync`,
`generate`, `generate-px4` — routes through it, so a new distro is one arm).
`ros-jazzy`/`ros-rolling` cargo features added across the 6 forwarding
Cargo.tomls (nros-rmw-zenoh → staticlib → nros-node → nros → nros-c → nros-cpp);
`nros-rmw-zenoh::keyexpr`'s "modern edition" branch (was `ros-iron`) now keys on
`any(ros-iron, ros-jazzy, ros-rolling)` so jazzy/rolling append the RIHS01
type-hash tail like iron; the `nros` umbrella mutual-exclusion `compile_error!`
covers all four. type_hash for iron/jazzy/rolling is the RIHS01 form with a
PLACEHOLDER digest (W1 computes the real one — the FORMAT is right, the digest
is not).

- *Verified:* `RosEdition` unit tests (parse/as_str/round-trip/type_hash for all
  four); `nros-rmw-zenoh --features ros-jazzy` compiles + keyexpr resolves;
  `nros --features ros-humble,ros-jazzy` fails with the mutual-exclusion error;
  rosidl-codegen (403) + nros-msg-to-idl parity (93) suites green; humble default
  byte-identical.

### W4 — multi-distro test infrastructure

- `scripts/ros/capture-edition-fixtures.sh` — capture Tier-A fixtures from an
  `osrf/ros:<distro>` container (RIHS01 hashes, `rosidl_adapter` IDL, `.msg`).
- Generalize `nros_tests::ros2`: replace the hard-coded `/opt/ros/humble`
  availability checks with the requested distro; add the Tier-B1 container-peer
  harness (or the B2 install probe), skipping when absent.
- RFC-0051 test matrix gains the **edition** axis; edition lanes are declared,
  precondition-skip when the distro is unavailable.

- *Accept:* an Iron or Jazzy interop lane PASSES against a container peer (B1) or
  an installed distro (B2), and skips loudly when neither is present; the
  offline Tier-A fixture tests run in CI with no ROS runtime.

**W4 STARTED (2026-07-25):**
- `scripts/ros/capture-edition-fixtures.sh <iron|jazzy|rolling>` — captures the
  Tier-A RIHS01 hashes (`ros2 interface hash`) + rosidl_adapter `.msg` from a
  throwaway `osrf/ros:<distro>-ros-base` container into
  `packages/testing/nros-tests/fixtures/ros-editions/<distro>/`. Syntax-checked +
  distro-guarded (humble/unknown rejected). Running it (a ~1 GB pull) produces
  the golden values **W1** asserts against — the capture is the W1 prerequisite.
- `nros_tests::ros2` is now distro-parametric: `is_ros2_distro_available(distro)`
  (bare-identifier-guarded) generalizes the hard-coded `/opt/ros/humble` check;
  `is_ros2_available()` keeps the humble default.

**W4 LANDED (2026-07-25) — offline fixture test + Tier-B1 container oracle.**

The interop-critical invariant is the keyexpr TYPE-HASH TAIL: a Jazzy peer and a
`ros-jazzy`-built nano-ros node interop iff their RIHS01 for the type matches.
Two tests pin it, split so the always-run half needs no ROS:

- **`edition_type_hash_offline.rs`** (rosidl-bindgen, CI, NO ROS) — drives the
  committed `fixtures/ros-editions/jazzy/{hashes.txt,srv-hashes.txt}` as DATA:
  the `rihs` engine reproduces the captured hashes for every type nano-ros can
  reconstruct (Int32, Header, Twist, SetBool + members). Re-capture updates the
  expected values automatically.
- **`edition_hash_oracle.rs`** (rosidl-bindgen, docker-gated Tier-B1 peer) — runs
  a live `ros:jazzy-ros-base` container as an oracle, reads the real RIHS01 a
  Jazzy node emits (share type-description JSON), asserts it still equals the
  committed fixture (a distro-DRIFT guard). Clean skip when docker/the image is
  absent. VERIFIED here: 7 committed Jazzy hashes current against the live
  container.

Chain: engine == fixture (offline) + fixture == live Jazzy (oracle) ⟹ engine ==
live Jazzy — i.e. nano-ros's keyexpr tail equals a real Jazzy node's.

**W4 REMAINING — LANDED (2026-07-27, via phase-311 W5).** The full nano↔Jazzy
WIRE lane now exists: `packages/testing/nros-tests/tests/ros_editions_zenoh.rs`
runs a `ros-jazzy`-built nano-ros zpico node ↔ a shared `rmw_zenohd` ↔ a stock
jazzy `rmw_zenoh_cpp` peer, proving end-to-end DELIVERY (not just the hash),
both directions, pub/sub + service + action-client (5/6; ROS→nano action server
is #0292). Both former blockers cleared: (a) the examples now build the jazzy
stack via the `ros-<edition>` passthrough feature (the RMW-mirrored selection
this W2 designed), and (b) the phase-311 `nano-ros-ros:jazzy` image carries
`rmw_zenoh_cpp` 0.2.9. The investigation (issue #0291) also PROVED the zenoh
transport is version-compatible (proto `0x09`, 1.7↔1.11) — the interop key is the
RIHS01 keyexpr tail this phase bakes, exactly as W1/W2 intended. Edition interop
lanes remain hand-picked cells (RFC-0051 §), NOT a 7th combinatorial matrix axis.

### W5 — encoding field per edition (coordinates phase-303 W1b)

Make the wire-encoding default a profile field the edition selects: `humble` →
XCDR1 (byte-identical to today, parity intact); `jazzy` → the phase-303 XCDR2
path (W2–W4 there). **Blocked** on the phase-303 W1 distro capture (RFC-0055
open-Q5 / #0267) — do not build the XCDR2 encoder before the live-peer evidence.

- *Accept:* deferred to phase-303 W1b acceptance; recorded here so the axis's
  encoding field is not forgotten.

**W5 STATUS (2026-07-25): still BLOCKED — no code this pass, by design.** The
encoding-profile field stays doc-only: adding a `RosEdition::wire_encoding()`
that returned `Xcdr2` for jazzy would MISREPRESENT capability (nano-ros serializes
XCDR1 only; the XCDR2 encoder is phase-303, gated on the #0267 live capture). The
axis's encoding dimension is recorded here + in RFC-0056 §"per-edition interop
profile"; it lands with phase-303 W1b, not before.

## Non-goals

- Runtime multi-edition in one binary (compile-time exclusive, like RMW/platform).
- Full package-set parity with every distro — nano-ros generates the interfaces a
  workspace declares, per edition.

## Done when

`[system].ros_edition` selects the edition end-to-end (codegen type_hash +
cargo feature + interface dir, W2); the RIHS01 computation matches captured
Iron/Jazzy references (W1); an Iron or Jazzy interop lane passes against a
container peer (W4/B1); and a `humble` build is byte-identical to today
throughout. The encoding field (W5) closes with phase-303.

## References

- Design: RFC-0056 (axis / profile), RFC-0055 (encoding), RFC-0051 (test matrix),
  RFC-0031 (declared-and-lowered selection — the model for W2).
- Phases: phase-41 (RIHS01 type hash — W1 drives it), phase-303 (XCDR2 encoding
  — W5 coordinates it).
- Issue: #0267 (the encoding half's root-cause + the demo-distro capture that
  unblocks W5).
- Research: `docs/research/rep-2011-type-hash.md` (RIHS01 canonical form).
