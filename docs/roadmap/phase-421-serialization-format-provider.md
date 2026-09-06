# Phase 421 — serialization format as a compile-time provider

**Status (2026-09-04). W1–W5 landed; W6 open.**
Implements [RFC-0088](../design/0088-serialization-format-is-a-compile-time-provider.md).
Depends on **[phase-420](archived/phase-420-package-identity-and-provider-format.md) W1
only** (the `<nano_ros_uses>` general consumption tag) — and only from W4 onward;
W1–W3 here are pure Rust and need nothing from that phase.

Related: RFC-0009 (bridge topic forwarding), RFC-0011 (uORB backend), RFC-0035
(RMW vtable ABI), RFC-0054 (C headers are the ABI SSoT), phase-325 (uORB interop
and bridge).

## Goal

Turn three unchecked prose claims into checked facts, and make a user's
serialization library a provider package.

The claims today:

- `EmbeddedRawPublisher::publish_raw` — *"Publish raw CDR-encoded data (must
  include CDR header)"*, checked by nothing.
- `nros-bridge` — *"both sides must use ROS-CDR … Cross-encoding bridges would
  need an explicit translator and are out of scope"*, checked by nothing.
- uORB, whose wire is the PX4 struct verbatim, treated as a special case rather
  than as a backend whose format differs.

The mechanism is compile-time, not runtime: ROS 2 uses format strings because
`rosidl_typesupport_c` resolves them through `dlopen`, and we do not `dlopen`.

## Work items

- [x] **W1 — the format is a compile-time fact.** (landed 2026-09-04)
      `nros_serdes::format` carries `SerializationFormatId` (`#[repr(u8)]`), the
      `SerializationFormat` trait and the `Cdr` / `Uorb` markers.
      `nros_core::RosMessage` gains a **defaulted** `SERIALIZATION_FORMAT_ID`;
      `nros_rmw::Session` gains `SERIALIZATION_FORMAT` + `_ID`, also defaulted;
      `nros_node::session` exposes `IMAGE_SERIALIZATION_FORMAT{,_ID}`.
      `format_check::{assert_message_format, assert_raw_format}` compare inside
      an inline `const {}`, called from the four typed entity-creation funnels.
      See "What landed differently" for why this is a const on `RosMessage`
      rather than an associated type on `schema::Message`.
      **Acceptance, met:** a `Uorb` message in a CDR image fails to build with
      `error[E0080]: evaluation panicked: message serialization format does not
      match the linked backend (RFC-0088)` plus the instantiation note naming
      the type — measured with a probe, which was then removed. No branch is
      added to the publish path: the whole effect is the `const {}`.

- [x] **W2 — the reserved vtable slot gets a body.** (landed 2026-09-04) `get_serialization_format`
      leaves the `identity` inert family in `check-rmw-slot-producers.py`; every
      backend returns its constant. Add the per-session accessors:
      `Session::serialization_format()` and `Node::serialization_format()` in
      Rust, `nros_node_get_serialization_format()` in C (cbindgen output), and
      `nros::Node::serialization_format()` returning `const char*` in C++.
      **Acceptance:** `check-rmw-api-parity` classifies the slot as produced; a
      two-backend image reports two different formats from two sessions.

- [x] **W3 — the bridge becomes the only runtime site.** (landed 2026-09-04) `RawSubscription::format()`;
      `PubSubBridge::new` returns `Result<_, BridgeError>` and refuses a
      mismatch; `SerializationFormatConverter` plus
      `PubSubBridge::with_converter` for the deliberate cross-format case.
      **Acceptance:** a bridge wired zenoh→uORB without a converter fails at
      construction with both format names in the error; the existing
      `declarative_bridge_zenoh_to_{cyclonedds,xrce}` cells stay green; the
      comparison is one byte, at construction, not per sample.

- [x] **W4 — the `serdes` provider family.** (landed 2026-09-04) One `FAMILIES` row in
      `check-provider-announcements.py`; `nros-serdes.toml` whose only field is
      `impl = "schema" | "codegen"` (absent file means `schema` and all
      defaults); `<nano_ros_provides kind="serdes" name="…"/>`; consumption via
      `<nano_ros_uses kind="serdes" name="…"/>` (phase-420 W1) and a
      `[deploy.<t>] serdes = "…"` key; `check-serdes-descriptors` covering S1–S4
      plus discriminant allocation.
      **Acceptance, partly met.** A provider in the USER WORKSPACE root — a
      directory outside this repo — is selected by name and resolves
      (`a_provider_in_the_user_workspace_reaches_the_default_search_path`). A
      provider at a *third* location is not reachable: `default_search_path`
      still returns two roots, which is phase-420 W6. No parser learned a new
      attribute, which is the half W1 of phase-420 bought.

      Two findings recorded rather than worked around:

      - **`[deploy.<t>].serdes` cannot exist.** `DeployBlock` is upstream
        (`ros-launch-manifest`, git tag) with `deny_unknown_fields`, so the key
        RFC-0088 D6 specified is a parse error, and adding it needs an upstream
        release plus a dependency bump. The ladder mirrors `rmw`'s minus that
        rung — and `rmw` on `[deploy.*]` is itself deprecated (issue 0951). D6 is
        amended to match.
      - **Issue 1054**: `provider_scan` reads `.nros-ignore` on the root it is
        handed, so scanning the nano-ros tree returns zero providers of ANY
        family. The marker's own header says the opposite is the contract, and
        `nros-pkg-index` honours it. Pinned by a characterization test here;
        the fix changes discovery for every family and belongs in its own
        change.

- [x] **W5 — the schema-driven strategy, with a reference provider.**
      (landed 2026-09-04) `nros_serdes::walk` carries `SchemaSerializer` plus the
      reusable walk (`SchemaSink` / `SchemaSource`, `encode_from_cdr` /
      `decode_to_cdr`). `packages/core/nros-serdes-packed/` is the reference
      provider: family `serdes`, `impl = "schema"`, `format_id = 3`,
      `<build_type>nros_cargo</build_type>`.
      **Acceptance, met with one signature amendment (below).** All 76 committed
      generated messages in `packages/interfaces/*` round-trip through the walk
      in BOTH CDR encodings — 152 (type, encoding) pairs, zero refusals, zero
      failures — using nothing but `&'static [Field]`
      (`nros-tests/tests/schema_serializer_round_trip.rs`). `corpus_is_exhaustive`
      re-counts `impl ::nros_serdes::Message` out of the sources at run time, so a
      message added later cannot sit outside the sweep. **No matrix cell was
      added** — see "What landed differently".

      Three findings recorded rather than worked around:

      - **D7's `*const u8` signature cannot be implemented over today's schema.**
        A `String` member's host type is `heapless::String<N>` and the schema
        carries no `N` (`FieldType::String` is the IDL type, and
        `BoundedString(n)` is the IDL bound, a different number); `heapless`'
        containers are `repr(Rust)`, so their interior layout is not a fact this
        crate may assume; and `NestedType` records no SIZE, so an
        `Array(N, Nested(..))` has no stride. `Field::offset` is sound — it only
        reaches the START of a container. The message side of both methods is
        therefore a `CdrReader` / `CdrWriter`, which makes `impl = "schema"` a
        **transcoder** strategy in v1 and is why the walk lives in `nros-serdes`
        rather than in each provider. Full reasoning in `walk.rs`'s module docs.
      - **A second parameter, `type_name`.** A `&'static [Field]` slice has no
        name — `Message::TYPE_NAME` is a separate const and nested members carry
        theirs in `NestedType` — so without it the top-level struct is the one
        node a self-describing format cannot name.
        `serialize_message::<M>` / `deserialize_message::<M>` supply both from a
        type.
      - **`wstring` reaches `SchemaError::Unsupported`, by name.** Not a limit of
        the walk: `CdrReader` / `CdrWriter` have no wide-string primitive in
        either direction, so there is nothing to transcode from. No message in
        `packages/interfaces/*` has one, so the sweep refuses nothing today — and
        the test treats a refusal as a FAILURE rather than a skip, so the first
        one that appears is read rather than tolerated.

- [x] **W6 — C and C++ assert at compile time.** (landed 2026-09-04) `NROS_SERIALIZATION_FORMAT_ID` /
      `NROS_SERIALIZATION_FORMAT` in the generated config; a per-message
      `_Static_assert` in C; `format_of<M>` plus `static_assert` in C++, using
      `const char*` rather than `std::string_view` so the header survives Zephyr's
      minimal libcpp. `check-format-macro-scope`: a bridge-linked image must not
      reference the macro, because a two-backend image has no single answer.
      **Acceptance, met.** `serialization_format_mismatch_probe.{c,cpp}` are
      expected-failure compiles in `check-c` / `check-cpp`; the C refusal names
      the message (`"RFC-0088: px4_msgs/VehicleStatus is not encoded in the
      format the linked backend speaks"`). `check-format-macro-scope` fails on a
      bridge-linked TU that references the macro, verified by injecting one.

      Landed differently in two places, both recorded here:

      - **The macro is lowered by cbindgen, not by the config header.**
        `nros_config_generated.h` is substituted in `nros-build-helpers`, and a
        build script cannot evaluate a Rust `const`. Each mirror instead carries
        a `const _` proving it equal to `session::IMAGE_SERIALIZATION_FORMAT{,_ID}`,
        so a backend that answers otherwise fails `cargo check` naming the drift.
        **The value's proper home is the RMW descriptor** (`[rmw]
        serialization_format` → `NanoRosRmwDispatch.cmake` → `configure_file`);
        the mirror is the affordable version, and its `const _` is the tripwire
        for the day a backend disagrees.
      - **`format_of<M>` defaults to `Cdr` rather than being a bare
        declaration**, mirroring RFC-0088 D1's defaulted const. Hand-written tag
        types pass through the typed creators (`DebugKeyValueTag` in the px4
        bridge, `FakeString` in a compat fixture); a hard-erroring primary would
        have broken them.

## What landed differently from the plan (2026-09-04)

- **W1's mechanism changed and RFC-0088 D1 was amended to match.** The format is
  a defaulted const on `nros_core::RosMessage`, not an associated type on
  `nros_serdes::schema::Message`. The associated type was implemented first
  (142 impls, 95 files) and reverted: `MessageForRmw` requires a schema only
  under `cfg(rmw_needs_type_descriptors)`, so the check was absent under zenoh,
  XRCE and uORB — the last being the backend it exists for. The const version
  touches no existing implementor and is universal.
- **The check is an inline `const {}`, not a where-clause bound** — `NodeHandle`
  is not generic over the backend. It is a `cargo build` error, not `cargo
  check`: the monomorphisation collector runs only during codegen, and the probe
  must be reachable (a private never-called fn is dropped first).
- **The raw path is an explicit `assert_raw_format::<F>()`**, because rustc
  forbids a defaulted type parameter on a function.
- **W2 found the uORB vtable truncated.** Its positional initializer stopped at
  `destroy_client`; reaching the new slot means naming 21 intervening slots,
  each written as the `nullptr` C++ was already giving it.
- **W2 also found `check-abi-bindings` had been silently skipping** — `bindgen-cli
  0.72.1` was not installed on the host, so the gate never ran.
- **W3's runtime path is not yet reachable.** Both `format()` accessors answer
  from the single image constant, so `PubSubBridge::new` cannot fail in a real
  image until a bridge image links two backends. The error rendering is tested;
  the end-to-end case waits on W4/W5.

- **W5 amended D7's signature and added no matrix cell.** The `*const u8`
  message pointer is not usable over today's schema (three measured reasons,
  above), so `SchemaSerializer` pivots on the CDR byte stream. And a
  `matrix::CELLS` row is FIXTURE-BACKED — `matrix_fixture_coverage.rs` G1–G4
  require a matching `examples/fixtures.toml` row — so a cell means an image.
  There is nothing yet to put in one: no backend declares `packed`, so the image
  would link a provider nothing calls, and the cell would answer "does dead code
  link" rather than "does the format work". The coordinate the walk varies over
  is the MESSAGE SHAPE, not platform × language × rmw × kind, and the sweep
  covers all 76 of those on the host with no fixture — so it runs in
  `just ci gate`'s `test-unit`, on every merge-queue batch, at no matrix cost.
  This is the RFC-0088 non-goal honoured rather than dodged: the axis stays at
  four, and a cell is the right answer only once a backend SPEAKS a second
  format (the uORB bridge, phase-325), which is where W3's runtime path becomes
  reachable too.
- **The reference format is `packed`, and it is a test vehicle, not interop.**
  Little-endian, no alignment padding, `u32` byte-length strings with no NUL, no
  DHEADER, `u32`-counted sequences. Every one of those differs from CDR
  deliberately: a walk that inherits CDR's padding, its `len + 1` string prefix,
  or its per-struct DHEADER fails the round trip instead of passing by
  coincidence. Nothing else speaks `packed` and nothing should.
- **`packed` gets no `SerializationFormatId` variant, on purpose.**
  `check-serdes-descriptors` S3 tolerates a `format_id` the enum does not name —
  it constrains descriptors that USE a reserved name or value and never demands
  one exist — so `format.rs` needed no edit. A variant would assert `packed` is a
  wire a BACKEND speaks, and none does. `SchemaSerializer::FORMAT_ID` is a raw
  `u8` for exactly that reason, which is also what RFC-0088 D2 says an
  image-local discriminant is.

## Sequencing

W1 → W2 → W3 are independent of phase-420 and can land immediately. W4 needs
phase-420 W1. W5 needs W4. W6 needs W1 and W2.

## Risks

- **Codegen touches every generated message.** W1 adds one associated type per
  message; regenerate and diff rather than hand-editing (`packages/interfaces/*`
  are committed).
- **The `u8` is image-local (RFC-0088 D2).** The temptation to treat it as a
  global registry will recur; the string is the cross-image identity, and a
  reviewer should reject any use of the discriminant in a file that outlives one
  build.
- **Schema-driven serialization is slower than generated.** Deliberate for v1;
  the answer is `impl = "codegen"`, which needs a codegen plugin ABI and is
  therefore out of scope here.
- **`can_loan_messages` (issue 0814) overlaps.** Loaning is the sanctioned
  zero-serialization path; this phase must not add a format check on a path that
  never serializes.

## Out of scope

Per-topic or per-publisher serializer selection (ROS 2 does not offer it
either); a codegen plugin ABI; a new matrix axis; converters as provider
packages.
