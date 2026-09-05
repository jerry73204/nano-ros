# Phase 420 — package identity and the provider format

**Status (2026-09-05). W1–W8 landed. W9 is the only work item still open,
and only its last step: its two source-list halves and its configuration
half are in review, and the single compile is argued down rather than
pending — see the item.** Implements
[RFC-0087](../design/0087-package-identity-and-provider-format.md). Sequenced
with [phase-421](phase-421-serialization-format-provider.md), which implements
RFC-0088 and needs **W1 of this phase only** — the rest of this phase can land
around it.

Related: RFC-0071 (provider descriptors), RFC-0062 (dependency SSoT),
phase-347 (RMW as a declared provider), phase-348 (source-time provider
discovery), phase-349 (platform family).

## Goal

Make the in-tree providers indistinguishable from a user's. Today
`packages/rmw/*` and the platform packages are found the way a user's package
would be — `provider_scan` says so explicitly — but they are *built* through a
build type that claims to be ament's, selected through a tag whose general form
does not exist, and described by descriptors that write convention out longhand.
Close those three, add the search path and selection verbs, and the provider road
is one road.

## Findings this phase acts on (measured 2026-09-04)

- Seven `<build_type>` spellings: `ament_cargo` 157, `ament_cmake` 125, `cmake`
  75, `ament_nros` 5, `nros_entry` 2, `nros_bringup` 1, `cargo` 1.
- `colcon-cargo-ros2/setup.cfg` registers 30 tasks keyed `ros.nros.<lang>.<platform>`
  and `nros_augmentation` gates on `desc.type.startswith("ros.nros.")` — **no
  `package.xml` declares that build type**, so the path cannot fire.
- `<nano_ros …/>` — 91 files (`deploy` 90, `rmw` 51, `board` 50), all consumption.
  `<nano_ros_provides …/>` — 52 tags in 21 files (board 33, rmw 11, platform 8).
  Exactly one file carries both: `nros-rmw-zenoh/package.xml`.
- Two readers have independently confused the two directions:
  `package_xml.rs`'s test message and `cmake/NanoRosPackageXml.cmake:41–46`.
- `default_search_path` returns exactly two roots, both inside the user's repo.

## Work items

- [x] **W1 — `<nano_ros_uses kind= name=/>` and one parser for three tags.** (landed 2026-09-04)
      Add the general consumption form; define `board=` / `rmw=` on the bare
      `<nano_ros …/>` tag as sugar for it; leave `deploy=` an attribute, because
      it names a `[deploy.*]` block and is not a provider kind. Implement the
      shared rule set once (must sit inside `<export>`, non-empty `kind`/`name`,
      comments stripped) and have both the Rust parser and
      `NanoRosPackageXml.cmake` consume that one implementation.
      **Acceptance, met:** a package selecting `kind="serdes"` resolves in both
      readers with no new attribute in either; the two reader-confusion tests
      still pass; `the_sugar_and_the_general_form_resolve_identically` asserts
      the equivalence in Rust and `check-package-xml-uses` asserts it in cmake.
      **This is what phase-421 W4 needs.**

      Landed as: one `read_announcement` helper serving both announcement tags
      in `package_xml.rs` (the two readers that confused the directions each
      implemented the rule separately — one match arm cannot disagree with
      itself); `PackageXml::{uses, deploy}` with `uses_of_kind()`;
      `NANO_ROS_EXPORT_USES_<KIND>` plus `NANO_ROS_EXPORT_USES_KINDS` in
      `NanoRosPackageXml.cmake`, fed by both spellings; and a buildless
      `cmake -P` gate. `deploy=` stays an attribute in both, and a
      `<nano_ros_uses kind="deploy" …/>` is not invented to carry it.

- [x] **W2 — `nros_cmake` / `nros_cargo` build types.** (landed 2026-09-04) Teach the reader both old
      and new, mapping `ament_cargo|ament_nros → nros_cargo` with a deprecation
      warning that names the file. Add `check-build-type-spelling`: the allowed
      set, plus RFC-0087 D2's class boundary — a provider, board or entry may not
      declare `ament_*`; an interface package may not declare `nros_*`.
      **Acceptance, met** (as a ratchet — the tree is not migrated until W3):
      the gate fails on a package that crosses the boundary in either direction,
      each rule watched failing on a constructed input.

      **The survey contradicted this item's premise: NOTHING read
      `<build_type>`.** Not `package_xml.rs`, not `NanoRosPackageXml.cmake`, not
      `nros-cli-core` — which only WRITES it. So "teach the readers both
      spellings" was really "give the readers a reader", and this wave adds one
      to cmake and one to `nros-cli-core`. It sharpens the Motivation's defect 2:
      colcon keys on a build type nothing declares, and no other consumer reads
      the field either.

      Three decisions worth carrying forward:

      - `ament_nros` maps to **`nros_cmake`, not `nros_cargo`** as this doc
        first said. All five in-tree uses are cmake-side — two carry a
        `CMakeLists.txt`, three are bringups that generate a CMake root.
      - **Only the three RETIRED spellings warn.** A deprecation on
        `ament_cargo` would fire on 148 in-tree packages and on every legitimate
        interface package, training people to ignore it before W3 can act.
        Whether `ament_cargo` is wrong depends on the package's CLASS, which is
        the gate's question, not a string's.
      - The gate grew a fourth rule, `owned-declares-nothing`: 34 owned packages
        (including all 21 providers) declare no `<build_type>` at all, and
        `catkin_pkg` then reports `catkin` — the same false ament-family claim,
        made by omission.

      Classification is from evidence, and `nros_generate_interfaces` is
      deliberately NOT ownership evidence: a message package calls it, and
      counting it would classify every user interface package as firmware. The
      gate cross-checks the two build-type tables against each other (S0) rather
      than being a third copy of the vocabulary.

      **Ordering hazard for W3/W4:** the scaffolder emitters
      (`emit_package_xml.rs`, `new_system.rs`, `scaffold.rs`) must not move to
      `nros_*` before W4 re-keys `colcon-cargo-ros2`, or a freshly scaffolded
      package becomes unbuildable by colcon. `ament_nros` is safe to move
      whenever — no colcon extension ever registered it.

- [x] **W3 — rewrite the nano-ros-owned packages.** (landed 2026-09-04) Entries,
      boards, RMW / platform providers, bringups. `packages/interfaces/*` and user
      message packages are **untouched** — they are ROS 2 packages. `ament_nros`,
      `nros_entry` and `nros_bringup` fold into the pair and declare no role.
      **Acceptance, met:** 371 `package.xml` rewritten (202 `nros_cmake`, 168
      `nros_cargo`, one held — below); the baseline shrank **301 → 1**;
      `check-build-type-spelling`, `check-package-xml-comments`,
      `check-package-xml-uses` and `check-provider-announcements` green.

      **The value follows the package's BUILD SYSTEM, not its old spelling** —
      the mapping table in `build_type.rs` / `NanoRosPackageXml.cmake`
      canonicalises a legacy *string*; it does not decide a *package*. Order of
      evidence used, per package: a bringup's declared images → the driver
      `plan::driver_for` picks; a descriptor-only provider → the path its
      contribution actually reaches an image by; otherwise `CMakeLists.txt`
      before `Cargo.toml`, because a package carrying both is CMake-rooted with
      the crate imported through corrosion.

      Consequences worth recording, because each inverts the obvious guess:

      - **19 Rust packages became `nros_cmake`** — the twelve Zephyr Rust
        leaves and six ThreadX ones, plus `mixed/src/rust_heartbeat_pkg`. Each
        carries a `CMakeLists.txt`; `west`/`cmake` is the build root and the
        crate is a staticlib it imports. Cargo is downstream of CMake there,
        not the entry point.
      - **7 bringups became `nros_cargo`**, including six that said
        `ament_cmake`. A bringup owns no build file at all; it generates a
        cargo root OR a cmake root per image, and an all-Rust, non-Zephyr
        workspace only ever takes the cargo one. Where a bringup's images span
        both drivers (`workspaces/rust`, `workspaces/realtime-rust`,
        `workspaces/mixed`, `workspaces/cpp`, …) **cmake wins**, for the reason
        `driver_for` gives: corrosion makes cargo consumable from cmake and
        nothing makes cmake consumable from cargo.
      - **One correction to W2's `ament_nros` note.** W2 recorded that all five
        `ament_nros` uses were cmake-side, "three are bringups that generate a
        CMake root". Measured per package, `multi_pkg_workspace_nuttx/src/demo_bringup`
        is not: its workspace is all-Rust, `driver_for("nuttx", false)` is
        `Driver::Cargo`, and the NuttX `apps/external` shim's `context::` rule
        shells `NROS_CARGO_BUILD`. It is `nros_cargo`. The four others are
        `nros_cmake` as W2 said. The *table's* `ament_nros → nros_cmake` row is
        unaffected — it canonicalises a retired string for a reader, and the
        sweep answers a different question.
      - **The three `ambiguous` packages W2 left to W3**, each decided on which
        half a consumer can actually use:
        `examples/native/c/custom-msg` → **`nros_cmake`** (application wins: it
        calls `nano_ros_add_executable`, and its messages come from
        `nano_ros_generate_interfaces`, not `rosidl_generate_interfaces`, so no
        ROS 2 node can consume them — the interface half is local to the
        example); `examples/native/rust/custom-msg` → **`nros_cargo`** (same
        shape, cargo-rooted); `nros-tests/bins/ros-edition-pose-pub` →
        **`nros_cargo`** (a `geometry_msgs` publisher fixture with no `msg/`,
        `srv/` or `action/` dir at all — its
        `<member_of_group>rosidl_interface_packages</member_of_group>` claims
        an interface it does not ship, and should be deleted separately).
      - **73 packages that said plain `cmake` / `cargo` were swept too.** D2's
        standalone-example exception does not reach them: they are workspace
        members that `find_package(nano_ros REQUIRED)` and call
        `nano_ros_auto_add_library` / `nros_components_register_node`. Plain
        `cmake` is the same false claim as `ament_cmake`, only quieter — stock
        colcon *does* register a `ros.cmake` task, so it attempts the build and
        fails at `find_package`, which is exactly the attempt-instead-of-refusal
        D2 exists to end.

      **Two standalone port templates keep plain `cmake`, deliberately** —
      `examples/templates/cpp-port-minimal-publisher` and
      `examples/templates/rclcpp-compat-smoke`. Both are verbatim stock ROS 2
      `ament_cmake_auto` packages whose whole subject is the size of the delta;
      making them declare a nano-ros build type would edit the thing being
      demonstrated. This is D2's standalone arm, used as written.

      **The one baseline row that remains is W4's, not a residue of judgement.**
      `examples/templates/local-msg-package/src/rust_consumer` is discovered by
      the `colcon build examples/templates/local-msg-package/src/` CI job and by
      `just colcon-parity`. On the CI runner (stock colcon, no
      `colcon-cargo-ros2`) it is already skipped with a warning, so nothing
      there changes either way; on a developer host *with* the extension it is
      built today by `ros.ament_cargo`, and rewriting it before W4 re-keys the
      entry points to `ros.nros_cargo` would silently stop building it. It moves
      in W4's commit, and the baseline empties there.

      **Deliberately not moved:** the scaffolder emitters
      (`emit_package_xml.rs`, `new_system.rs`, `scaffold.rs`), for the same
      reason — a freshly scaffolded package must stay colcon-buildable until W4.

- [x] **W4 — re-key the colcon extension.** (landed 2026-09-04) Entry points
      became `ros.nros_cargo` / `ros.nros_cmake`; the 30
      `ros.nros.<lang>.<platform>` keys and the `startswith("ros.nros.")` gate
      are gone. **Acceptance, met**, measured with the host's own colcon 0.20.1
      on a two-package workspace: `colcon list` reports `(ros.nros_cargo)` /
      `(ros.nros_cmake)`; with the extension registered `colcon build` selects
      `NrosBuildTask` for both, and a full cargo-path build installs the binary,
      the `package.xml` and the ament index marker, with `colcon test` running
      it through `NrosTestTask`; with the extension absent the same workspace
      reports `No task extension to 'build' a 'ros.nros_cargo' package` and
      installs nothing.

      **The 30 keys became 2, and the two facts they used to carry moved to
      where they live.** Deleting the `ros.nros.<lang>.<platform>` string
      deleted the only place `lang` and `platform` were ever written down, so
      each had to be re-sourced (`colcon_nano_ros/manifest.py`, one reader for
      both):

      - **build system ← `<build_type>`**, which is that field's whole job. It
        is strictly better than the retired `lang` token: W3 measured 19 *Rust*
        packages that are `nros_cmake`, and `lang == "rust"` would have routed
        every Zephyr and ThreadX leaf to `cargo build` instead of west/cmake.
      - **platform ← `<export><nano_ros deploy=…/></export>`**, the RFC-0087 D3
        consumption tag, with absent meaning the host — the identical rule
        `_nros_deploy_to_platform` already applies in
        `cmake/NanoRosPackageXml.cmake`, not a new default invented here.
      - **language is not recoverable, and was not faked.** `nros_cmake` says
        CMake; nothing in any manifest says C versus C++. It turned out nothing
        needed it: `_build_cmake` never read its `lang` argument, and the
        augmentation's `_needs_c` / `_needs_cpp` were written and never read
        (C/C++ bindings are CMake's `nano_ros_generate_interfaces()` job). The
        one load-bearing question — does this workspace need Rust bindings —
        is answered from evidence instead: a `nros_cargo` package always does,
        and a `nros_cmake` package does when it carries a `Cargo.toml`.

      **A silent wrong answer became a refusal.** `PLATFORM_TARGETS.get()`
      returns `None` for an unmapped platform, and `None` is also the spelling
      for *native*, so a package deploying somewhere the task cannot
      cross-compile to would have built a host binary and reported success.
      The cargo path now names the value and the known set and returns 1.

      **The scaffolder was the one real producer of the dead spelling.**
      `cargo-nano-ros/src/scaffold.rs` emitted `nros.<lang>.<platform>` for
      Rust — so the 30 keys were reachable in principle by a scaffolded
      package, just never by a tracked one. Moving it to `nros_cargo` forced a
      second change: the `<nano_ros deploy= rmw=/>` tuple is now emitted for
      **every** language, not only C/C++, because the platform no longer rides
      in the build type and a `--platform freertos` Rust package that declared
      no `deploy` would build a host binary. `scaffold_component_rust` also
      gained an `<export><build_type>` — it emitted none at all, which
      `catkin_pkg` reports as `catkin`, the `owned-declares-nothing` claim by
      omission that W2 named, and its C and C++ siblings already declared one.

      **W3's three held-back items moved here, in this wave's commit:**
      `examples/templates/local-msg-package/src/rust_consumer/package.xml`
      (`ament_cargo` → `nros_cargo`), which empties
      `scripts/build-type-spelling-baseline.json` to `{}` — the gate now reads
      407 `package.xml` with **zero** grandfathered rows; the scaffolder
      emitters `emit_package_xml.rs` (component `nros_cargo`, bringup
      `nros_cmake`), `new_system.rs` (`ament_nros` → `nros_cmake`) and
      `scaffold.rs`; and this doc's status line. `just colcon-parity` still
      passes (3 packages finished, `install/lib/consumer/consumer` produced):
      its assertion is on the `ament_cmake` C++ `consumer`, and `rust_consumer`
      is skipped by a stock colcon exactly as it already was on the CI runner,
      which installs no `colcon-cargo-ros2`.

      **`scripts/docs/migrate-example-cmake-ament.py` was updated, not
      deleted.** Its `package.xml` half wrote `<build_type>cmake` →
      `ament_cmake`, which W3 made backwards; it now writes `nros_cmake`. The
      CMake shape it emits is unchanged — `find_package(nano_ros)` +
      `ament_package()` is still RFC-0048's ament *shape*, and only the
      ownership claim moved. Deleting it was the other option, and lost:
      `packages/testing/nros-tests/tests/example_shape.rs` still names it as
      the remedy for a leaf carrying a superseded CMakeLists, so it has to keep
      working. A dry run over the 27 native leaves reports 27 already-ament, 0
      migrated — a no-op, as it should be.

      **Not done: the drift issue.** This item asked for one filed first, so
      the history records the path was dead rather than merely renamed.
      `just issue-new` reserves an id by pushing a ref to origin, which this
      change's session could not do; the record lives here and in
      `colcon_nano_ros/manifest.py` instead.

- [x] **W5 — descriptor derivation.** (landed 2026-09-04) Derive names (from
      the announcement), cargo feature, cmake value, C define token, cffi
      feature and crate. `check-derived-descriptor-fields`: a stated derivable
      field must equal its derived value — a ratchet, so history's spellings are
      grandfathered and new drift is refused. New descriptors state only
      non-derivable facts; an absent descriptor means every default applies.
      **Acceptance, met:** the five derivable fields were deleted from ALL FOUR
      rmw descriptors (20 lines) and the generated `rmw_table.rs` is
      byte-identical — `rmw_cmake_dispatch_is_current` still passes against the
      committed `cmake/NanoRosRmwDispatch.cmake`, which is the same claim one
      lowering further down.

      **`crate` is the one field that resisted, and the RFC's table is wrong
      about it.** D4 says it derives from "the package's `Cargo.toml`". Measured
      across the four backends, that is right for one and a half:
      `nros-rmw-zenoh` names its own crate; `nros-rmw-xrce` ships **no
      `Cargo.toml` at all** and its `[rmw.provides.cargo].crate` names a SIBLING
      package (`nros-rmw-xrce-cffi`, the cffi shim beside it);
      `nros-rmw-cyclonedds` states `sys_crate = "cyclonedds-sys"`, also not its
      own; `nros-rmw-uorb` is C++ and states neither. A rule that holds for two
      of four is a convention with exceptions, which is not a convention — so
      `crate` stays authored, and the xrce row is the ratchet's one
      grandfathered entry rather than a fact explained away. (`cpp_define` never
      entered the derived set; its own comment says why.)

      **`names` is the ONLY field the announcement can source, and only for a
      single-entry package.** Boards are the counterexample:
      `nros-board-nuttx-qemu` declares two `[[board]]` entries and announces
      seven names in one flat list, and `<nano_ros_provides>` carries no
      boundary that could say which four are the ARM variant's. So **rmw went
      nameless and board did not**, and `board_crate` — which every board that
      states one states correctly — is deletable only once the reader in
      `nros-cli-core/orchestration/board_descriptor.rs` derives it. That is a
      one-crate change this wave did not own; the gate covers the field
      meanwhile, so a wrong `board_crate` fails now even though a redundant one
      is still tolerated.

      **`check-provider-announcements` grew A2n rather than a second
      mechanism.** phase-421 W4's `(glob, extract)` row shape already carried
      `extract=None`; W5 gives it a meaning that binds: a nameless family's
      descriptor must declare NO names anywhere, searched over the whole
      document rather than one known table. Deleting A2's comparison without
      that would have left `names` re-addable with no reader and no gate —
      worse than a disagreement, because it can be edited with no symptom.

      **The derivation has one implementation and three readers, cross-checked
      rather than trusted.** `cargo-nano-ros/src/derived_descriptor.rs` is
      compiled into both the library and `build.rs` (`#[path]`), so the
      generator cannot be a second spelling. Its cheap announcement scanner —
      a build script may not gain a dependency without moving `Cargo.lock` — is
      asserted equal to `package_xml::PackageXml` (quick-xml) on every in-tree
      backend by `descriptor_names_come_from_the_package_xml_reader`. The gate's
      Python copy is a checker, never a producer. That is the shape CLAUDE.md
      demands after the rmw parity map and the vtable sat green and disagreed
      by 25 symbols.

      **Two gates outside the wave's file list had to move with the schema**,
      recorded here because a gate left behind is how a schema change becomes a
      red main: `check-rmw-descriptors` required the four derivable fields to be
      PRESENT (S1) and read `names` out of the descriptor (S2/S3), so it failed
      on all four descriptors the moment they shrank. S1 now requires
      `cpp_define` alone, S2 claims names from the announcement, and S3 is
      retired — "the canonical name is the first one" became structural when
      `build.rs` started taking the first announcement.

- [x] **W6 — the search path.** (landed 2026-09-04, `8a96d6d77`)
      `[workspace] package_paths` in `nros.toml` plus `NROS_PACKAGE_PATH`,
      nano-ros tree first, shadowing **reported**: `nros ws packages` prints
      each package's kind, its root, and what it hid.
      **Acceptance, met:** a provider in an out-of-repo root is selected by
      name, and a same-named provider in two roots produces a printed shadowing
      report rather than a silent winner. `RootOrigin` names which of the three
      sources contributed a root, so the report says WHERE a winner came from
      and not merely that it won; `ProviderResolution::shadowed` retains the
      losers rather than discarding them, and a root that is not a directory
      prints a MISSING marker instead of being silently skipped.

- [x] **W7 — selection verbs.** (landed 2026-09-04) `nros build
      --packages-select` / `--packages-up-to`, colcon semantics, over the
      existing topological order.
      **Acceptance, met:** `up_to_narrows_the_generated_cargo_root_to_the_closure`
      and `a_selection_narrows_the_generated_cmake_root` assert the closure and
      the "nothing else" on both root-emitting drivers.

      Landed as one pure function, `builder::discover::select(&Discovered,
      &Selection)`, applied in `plan_builds` as stage 1b. **It filters the order
      stage 1 already returned and adds no second sort** — a subset of a
      topological order, taken in place, is a topological order of the subset,
      and `a_selection_keeps_the_topological_order_it_was_given` asserts the
      filter does not disturb it. `topological_order`'s output expressed the
      filter with nothing missing.

      Three decisions, two of them divergences from colcon, both toward failing
      instead of continuing:

      - **An unmatched name is an ERROR, not colcon's warning.** colcon can be
        agnostic because the name might legitimately live in an install prefix.
        D8 says nano-ros has none — the selection resolves against the source
        tree and nothing else — so an unmatched name is a typo or a stale
        script, and warning past it narrows the build to something nobody asked
        for and then reports success. `plan::resolve` already answers an unknown
        IMAGE this way, with the available names in the message; this is the
        same answer one noun over.
      - **An incomplete selection is REFUSED.** This is the wave's headline
        question and colcon's answer does not port. `--packages-select A` where
        A needs B is colcon's way to rebuild one package against an existing
        install; here there is no install, one merged root, and per-target
        static objects — B would simply be absent from the generated
        `[workspace] members` / `add_subdirectory` set, and the failure would
        surface a layer down as an unresolved path dependency or a missing CMake
        target, which is an error about the wrong thing. So the `<depend>`
        closure of the FINAL set is checked, the hole is named, and
        `--packages-up-to <the same names>` is offered as the fix. The check
        runs over the final set rather than over `--packages-select` alone
        because an up-to closure is complete by construction and intersecting it
        is not (`an_intersection_that_punches_a_hole_is_still_refused`).
      - **The two flags compose as their INTERSECTION**, which is colcon's own
        composition — each is an independent deselecting filter — and the only
        one under which adding a flag can never widen a build. A disjoint pair
        is an error naming the composition rather than an empty build.

      Two seams worth recording, because each is a place the obvious placement
      is wrong:

      - **Images are collected BEFORE the selection is applied.** An image is
        declared by a bringup's `system.toml` and is a property of the
        workspace, exactly as the generated cargo root's member list is
        (phase-383 W9.b's reasoning). Narrowing first makes
        `--packages-select talker_pkg` answer "this workspace declares no
        `[image.*]`".
      - **`check_declared_depends` keeps the UNNARROWED set.** It walks the
        whole tree itself for `<depend>` declarations, so handing it the
        narrowed name set would report every deliberately-dropped package as an
        unresolved dependency.

      Known blind spot, stated rather than papered over: the closure check sees
      the `<depend>` graph, not a cargo `path` dependency between two crates
      that declare nothing in `package.xml`. A `cargo_only` member carries no
      `depends` by design ("cargo resolves its own dependency order from
      `Cargo.toml`"), so dropping one is still loud — the noise just comes from
      cargo rather than from here.

      One consequence for a cargo workspace: a narrowed selection narrows the
      generated root's member list, which changes `Cargo.lock`. Where `--locked`
      is injected (the repo's `scripts/bin/cargo` shim) that is a hard error
      rather than a silent re-resolve, which is the behaviour issue 0359 wants.
      Generated entry packages are build output of the images being built, not
      discovered packages, so they are not selectable and are not narrowed.

- [x] **W8 — the fetch-pin gate.** (landed 2026-09-05, `207519a70`)
      `check-vendor-fetch-pinned`:
      every `FetchContent_Declare` / `ExternalProject_Add` in a discovered
      package carries `URL_HASH`, and any downloading build script verifies a
      digest. Then convert **one** `nros-sdk-index.toml` `[source.*]` row into a
      vendor package as proof, choosing a row whose pin is a lag rather than a
      decision — never cyclonedds or zenoh-pico, whose pins are decisions
      (RFC-0075, issue 0507).
      **Acceptance:** the converted dependency is reached by `<depend>` on the
      vendor package's name, its values arrive through CMake targets or
      `DEP_<LINKS>_<KEY>` rather than ambient environment, and the old
      `[source.*]` row is deleted in the same commit.

      **Amended 2026-09-05: the "prove it by converting one" half is RETIRED,
      and W8 is the gate alone.** RFC-0087 D5 now says a vendor package nothing
      depends on is a fixture, not a proof — converting a row no consumer asked
      to move would demonstrate the shape against a package written to
      demonstrate the shape, which is the vacuity this repo gates for elsewhere
      (`check-no-vacuous-tests`). W9's survey then found the stronger form of
      the same objection: for a PATCHED upstream a fetch is impossible, and for
      an unpatched one it trades a gitlink for a weaker pin and leaves the
      duplicated build — the actual defect — untouched. So the first conversion
      waits for a consumer that needs one; the Orin SPE BSP (phase-418) is the
      candidate, being a clean upstream tarball with a real dependent.
      The gate ships with no subject and says so on every run, which is the
      honest state for a rule whose first case has not arrived.

- [~] **W9 — the in-tree vendored backends adopt the same shape.** (surveyed +
      partially landed 2026-09-05) `zpico-sys` and `xrce-sys` currently vendor
      through a submodule plus `build.rs`, which is a third mechanism. Split each
      into a vendor package (fetch/build of the upstream tree) and a provider
      package (the backend), so ours and a user's differ in nothing but location.
      Largest wave; do it after W8 has proven the shape on a simpler row.
      **Acceptance:** `zenoh-pico` and Micro-XRCE-DDS are reached by package
      name; `check-submodule-pins` still governs whichever remain submodules; no
      backend keeps a bespoke vendoring path.

      **The survey moved the wave, and two of this item's premises are false.** What landed is identity for the one directory that already IS a
      vendor package; the conversion this item names is refused, with reasons.

      **S1 — `xrce-sys` has no `build.rs`, and no crate.** phase-321 W1.d deleted
      the `xrce-sys` crate (701 LoC + a 307-line build script) for having zero
      dependents, leaving a directory holding two submodules and a README that
      says so: *"this directory is a SUBMODULE HOST, not a crate … Do not re-add
      a crate here."* So "vendor through a submodule plus `build.rs`" describes
      only `zpico-sys`.

      **S2 — the bespoke path is not the submodule, it is that every vendored
      tree is compiled TWICE, once per language lane.** Measured:

      | tree | cargo lane | cmake / west lane |
      | --- | --- | --- |
      | zenoh-pico | `zpico-sys/build.rs` → `nros-zpico-build` (2,593 LoC) | `zephyr/cmake/nros_rmw_zenoh.cmake:11` — a `GLOB_RECURSE` over the same submodule |
      | micro-XRCE + micro-CDR | `nros-rmw-xrce-cffi/build.rs:160-245` | `nros-rmw-xrce/CMakeLists.txt:143-193` — a hand-copied source list |

      The XRCE pair is the sharpest case: its lockstep is asserted by a comment
      calling it "a 115.K.2 invariant", and until this wave that comment named
      `packages/rmw/xrce/xrce-sys/build.rs` — **the file phase-321 W1.d deleted.**
      The invariant has been pointed at a ghost since phase-321.

      **S3 — the mirror has already drifted, in two respects.**
      `nros-rmw-xrce-cffi/build.rs` honours `NROS_LINK_IP=0` (phase-204.7) and
      drops `udp_transport{,_posix}.c`; `nros-rmw-xrce/CMakeLists.txt` compiles
      them unconditionally, so the CMake lane cannot build a serial-only XRCE
      node. **FIXED 2026-09-05 (issue 1068, archived).** The two lists became
      one — `packages/rmw/xrce/xrce-sources.txt`, with the conditionals carried
      as named groups so a lane supplies only a boolean per token and never
      decides which files a token covers. Neither lane names a `.c` any more,
      and `check-xrce-source-manifest` asserts that plus token agreement in
      BOTH directions, because a token only one lane answers is this same
      defect one conditional over. And the vendored VERSION is restated by hand in four places, only one of
      which agrees with the tree it describes: the client gitlink is upstream **v3.0.1**
      (`bdfa2809` = "Release v3.0.1") and micro-CDR is **v2.0.2**, while
      `nros-rmw-xrce-cffi/build.rs:359-383` bakes `2.4.1` (wrong) and `2.0.2`
      (right), `nros-rmw-xrce/CMakeLists.txt:59-62` bakes
      `2.4.1` for BOTH — `PROJECT_VERSION*` is never reset between the two
      `configure_file` calls, so `MICROCDR_VERSION_STR` compiles as `"2.4.1"` in
      the CMake lane and `"2.0.2"` in the Rust one — and `[source.micro-xrce-dds-
      client] version` in `nros-sdk-index.toml` says `2.4.3-nros1`. Nothing reads
      those macros (grepped: zero uses in either vendored tree and in ours), so
      the drift is cosmetic today; it is left ALONE on purpose, because the fix
      is to derive the version from the tree that has it, not to correct four
      literals and leave five spellings.

      **S4 — converting either to a fetch is strictly worse, and for zenoh-pico
      it is impossible.** `zenoh-pico` is a PATCH LINE: `jerry73204/zenoh-pico`
      branch `nano-ros`, pinned `c5853157`. D5's worked example is `URL` +
      `URL_HASH`, and a tarball has nowhere to put patches; a `PATCH_COMMAND`
      would resurrect the `patches/` directory `.gitmodules` records as
      deliberately deleted for the qemu fork in favour of the fork's own history.
      The remaining option, `GIT_TAG <full sha>` against our own fork, satisfies
      `check-vendor-fetch-pinned` but is the identical pointer with
      `check-submodule-pins` removed — W8's own argument for converting no row.
      The eProsima pair IS unpatched upstream (no `branch =`, upstream URL, both
      on release commits), so a fetch is mechanically possible there — and still
      buys nothing: it trades a gitlink for a weaker pin, drops the trees out of
      `nros setup --source`, and leaves the duplicated build untouched, which is
      the actual defect.

      **Decision: a fork IS a vendor package whose "fetch" is a submodule.** That
      is the finished shape, not a way-station. RFC-0087 D5 should say so — it
      currently reads as though `FetchContent` were the only spelling of "fetches
      an external source tree", and a gitlink is the strongest of the three pin
      forms, not an unmigrated one. **Recommended amendment to D5, not made here
      (`docs/design/` was outside this change's ownership):** add that a vendor
      package's fetch may be a submodule, that a PATCHED upstream must be one,
      and that `check-submodule-pins` and `check-vendor-fetch-pinned` partition
      the surface rather than ranking it.

      **Landed:**

      - `packages/rmw/zenoh/zpico-sys/package.xml` — the zenoh-pico vendor
        package gets an identity. It already owned the fetch (the gitlink), the
        build (the C compile for six platforms) and the export (`links = "zpico"`
        → `DEP_ZPICO_*`, which is D5's cargo channel, live since phase-214). The
        only thing missing was that it was not a package, so nothing could name
        it. `nros_cargo`, no `<nano_ros_provides>` — a vendor package is not a
        kind (D5).
      - `nros_rmw_zenoh` gains `<depend>zpico_sys</depend>`, so the vendored tree
        is reached BY PACKAGE NAME. Inert by construction: `topo_inner` filters a
        `<depend>` naming a package outside the scanned set
        (`provider_scan.rs:438`), and `check_declared_depends` reads only the
        workspace being built, never the nano-ros tree.
      - `nros-rmw-xrce/CMakeLists.txt` — the mirror comment now names the file
        that exists, and records both measured divergences instead of asserting a
        lockstep that does not hold.

      **Not done, and why:** `xrce-sys` gets NO `package.xml`. It builds nothing —
      its two consumers each build its contents — so any `<build_type>` it
      declared would be the same false claim D2 exists to delete, and a vendor
      package nothing builds is W8's fixture. (Checked: neither eProsima tree
      ships a `package.xml` of its own, so there is no discovery-shadowing reason
      to add one either.) XRCE's vendor package cannot be created by adding a
      file; it needs the two copies of its build to become one.

      **W9 remainder, the real content:** give each vendored tree ONE build with
      ONE source list, consumed by both lanes.
      - XRCE: a vendor build under `xrce-sys/` producing `microxrcedds_client` +
        `microcdr` targets and a cargo `links` crate, with the source list and the
        version read from the vendored tree; `nros-rmw-xrce` and
        `nros-rmw-xrce-cffi` consume it instead of listing sources. This is what
        closes "no backend keeps a bespoke vendoring path", and it re-adds the
        crate phase-321 W1.d removed — legitimately, since this one has two
        dependents, which is exactly what that deletion said it lacked.

        **Scoped 2026-09-05, after issues 1068 and 1069 removed two of the four
        mirrors.** What the two lanes still hold in duplicate, measured:

        1. **The config-header generation.** `nros-rmw-xrce-cffi/build.rs` has
           `generate_ucdr_config` / `generate_uxr_config`, which read the
           upstream `config.h.in` templates and substitute by hand;
           `nros-rmw-xrce/CMakeLists.txt` runs `configure_file` on the same two
           templates. Two implementations of "fill in this template", and the
           token VALUES are restated in both — MTUs (4096 / 512), the
           `UCLIENT_PROFILE_*` flags, `XRCE_STREAM_HISTORY`, the `XRCE_MAX_*`
           entity caps.
        2. **The knob resolution, and this half is already BROKEN.** `zephyr/
           Kconfig` defines six `CONFIG_NROS_XRCE_*` knobs — `TRANSPORT_MTU`,
           `MAX_SUBSCRIBERS`, `MAX_SERVICE_SERVERS`, `MAX_SERVICE_CLIENTS`,
           `BUFFER_SIZE`, `STREAM_HISTORY`. The cargo lane reads them (via env +
           `$DOTCONFIG`); **the CMake lane reads zero of them** (`grep -c
           CONFIG_NROS_XRCE nros-rmw-xrce/CMakeLists.txt` = 0) and hardcodes the
           defaults instead. Kconfig line 992 says so out loud — "read by
           nros-rmw-xrce-cffi/build.rs" — naming one lane. So an image that sets
           `CONFIG_NROS_XRCE_TRANSPORT_MTU=1024` gets 1024 in the TUs cargo
           compiles and 4096 in the TUs cmake compiles. That is issue 0460's
           class in the mirror direction, and it is not hypothetical.
           **Open question to settle FIRST, because it decides the severity:**
           can one image contain TUs from both lanes? If yes this is also an
           0135-class ABI split — flag-gated struct layouts disagreeing inside a
           single link — and `examples/workspaces/mixed` is the entry that would
           catch it. If no, it is "the C lane ignores its own Kconfig", which is
           merely wrong rather than corrupting.
        3. **The compile itself.** `cc::Build` → `nros_rmw_xrce_c_inline` on one
           side, `add_library(nros_rmw_xrce STATIC …)` on the other, over the
           same TUs from `xrce-sources.txt`, with each lane choosing its own
           flags and defines.

        So the remaining work is not "share a list" — 1068 did that — it is
        **one build with one set of resolved knobs, consumed twice**. Sequence:
        settle the open question in (2) first, since a positive answer makes
        this urgent rather than tidy; then hoist the knob resolution to one
        place; then the template substitution; then the compile. (Followed:
        the question came back NO, and 1 + 2 landed together — they are one
        change, because the knob ladder's whole output is the template
        substitution. See below.)

        **Steps 1 and 2 LANDED 2026-09-05, and the open question is
        SETTLED: NO.**

        **The measurement.** One image cannot contain TUs from both lanes, so
        this was never an issue-0135 ABI split. Evidence, read out of the build
        graph rather than inferred from directory names:

        - nothing in the tree `add_subdirectory()`s or `find_package()`s
          `packages/rmw/xrce/nros-rmw-xrce`. The ONLY configure of it is
          `just check rmw-xrce` (`just/check.just`), standalone;
        - after that configure, `cmake --build … --target help` lists four
          targets, and `tests/CMakeFiles/nros_rmw_xrce_c_smoke.dir/link.txt`
          reads `… ../libnros_rmw_xrce.a ../nros_platform_impl_build/
          libnros_platform_posix.a -lrt` — the archive reaches that project's
          own two CTest binaries and nothing else;
        - `zephyr/cmake/nros_rmw_xrce.cmake` is an explicit no-op ("Nothing to
          compile here anymore"), so the Zephyr XRCE path compiles no C through
          cmake at all;
        - `cmake/NanoRosRmwDispatch.cmake` maps `xrce` to `NROS_RMW_RLIB_DEP
          nros-rmw-xrce-cffi` — the CARGO lane — for every image, and
          `CMakeLists.txt:268` bundles that Rust backend into the umbrella;
        - the one file that names a CMake package for this backend,
          `packages/api/nros-c/cmake/NanoRosLink.cmake`, is included by NOTHING
          (the live `cmake/NanoRosLink.cmake` never mentions XRCE) and the
          `NrosRmwXrceConfig.cmake` it would `find_dependency` does not exist —
          phase-140 deleted the install rules.

        So the severity is **test fidelity, not corruption**: the CMake lane's
        CTest harness was validating the backend at values no image compiles
        once a knob is set. It is also NARROWER than this item assumed — the
        CMake lane never participates in a Zephyr build, so those six Kconfig
        options were not merely ignored there, they were absent. **No issue
        filed**: the divergence is fixed rather than tracked.

        **What landed.**

        - `packages/rmw/xrce/xrce-config.txt` — the sibling of
          `xrce-sources.txt`. That one answers "which files"; this one answers
          "with what values". Four record types (`value`, `knob`, `flag`,
          `define`), the same line-oriented dependency-free format, and the
          SAME condition vocabulary — because "which files" and "which profile
          defines" have to agree or the header promises a profile whose TUs
          were not compiled. `never` is its one added token, for a
          `#cmakedefine` that is off on every target; stated rather than
          omitted, because an omitted toggle is `/* #undef */` under
          `configure_file` and an UNTOUCHED `#cmakedefine` line under a hand
          substitution — one silence, two different headers.
        - `build.rs` lost `generate_ucdr_config` / `generate_uxr_config` for a
          single `generate_config`, and both MTU consts; `CMakeLists.txt` lost
          its `set(UCLIENT_…)` block. Neither lane states a value now.
        - ONE knob ladder, implemented twice against one statement:
          `KnobResolver` in `build.rs` and `_nros_xrce_knob()` in the
          CMakeLists. Rungs 1 (env), 2 (`CONFIG_<env>` in `$DOTCONFIG`) and 4
          (the manifest default) are identical, including the treatment of the
          `-1` DERIVE sentinel as "nothing stated". Rung 3 (`[knobs.xrce]`) is
          cargo-only and SAID SO in the manifest header rather than left for a
          reader to assume symmetry: it needs a TOML parser the CMake lane
          cannot grow without the dependency the format exists to avoid, it
          covers two knobs, and it cannot be delivered to a lane with no cargo.
        - `zephyr/Kconfig` — two stale `Maps to` lines corrected.
          `NROS_XRCE_TRANSPORT_MTU` said "Maps to XRCE_TRANSPORT_MTU", a macro
          that does not exist; `NROS_XRCE_STREAM_HISTORY` named the environment
          variable and said "read by nros-rmw-xrce-cffi/build.rs", naming one
          lane. Those lines are now a CONTRACT (see the gate).

        **Verified, not asserted.** Both lanes were configured/built and their
        generated headers diffed, three ways: at defaults (byte-identical, AND
        byte-identical to the pre-change baseline — no behaviour moved); with
        `NROS_XRCE_TRANSPORT_MTU=1024 NROS_XRCE_MAX_SUBSCRIBERS=2
        NROS_XRCE_STREAM_HISTORY=8` in the environment (identical, and both
        lanes emit exactly `-DXRCE_MAX_SUBSCRIBERS=2 -DXRCE_STREAM_HISTORY=8`);
        and with those stated in a `$DOTCONFIG` instead (identical, `768`
        landing in both lanes' `UXR_CONFIG_UDP_TRANSPORT_MTU` where the CMake
        lane previously compiled 4096 whatever Kconfig said). `just check
        rmw-xrce` still passes 2/2.

        **Gate: `just check xrce-config-manifest`**
        (`scripts/check-xrce-config-manifest.py`). It checks the WIRING, not
        only the shape, because a row whose columns are all valid and whose env
        name points at the wrong knob is the failure a shape check cannot see:
        each `CONFIG_NROS_XRCE_*` option's Kconfig `Maps to <SYMBOL>[,
        <SYMBOL>]…` line must name EXACTLY the symbols the manifest binds that
        knob to. It also asserts both upstream templates are fully covered in
        both directions, that neither lane states a value of its own, and that
        every knob `nros_cargo_build.cmake` forwards is bound — all six, not
        the one in the example.

        Three existing gates moved with it, each because this change broke
        them: `check-xrce-source-manifest` now takes its condition vocabulary
        from BOTH manifests (a lane answering `never` otherwise reads as dead
        selection logic); `check-xrce-vendored-versions` learned the single
        `generate_config`, and its three crosswire vectors survived the move —
        the pairing got EASIER to check, since the template path and the
        `vendored_project_version()` argument now sit in one call expression;
        and `check-kconfig-knob-forwarding` gained a `MANIFEST_READERS` arm,
        since a reader may now name its knobs in a shared manifest rather than
        in its own source. That last one is load-bearing and is mutation-tested
        both ways: dropping a `define` row reports the forwarded knob as
        unread, and a reader that stops PARSING the manifest reports the
        manifest as unconsumed.

        **Step 3 (one compile) NOT attempted, and this item's sketch of it is
        wrong in one respect.** It reads "a cargo `links` crate … consumed by
        both `nros-rmw-xrce` and `nros-rmw-xrce-cffi`". `nros-rmw-xrce` is not
        a cargo crate — it is a CMake project — so it cannot consume a cargo
        `links` crate or its `DEP_<LINKS>_*` at all. "One compile consumed by
        both" therefore has to be a CMake project plus a cargo crate that
        drives it (Corrosion, or a build script that shells to cmake), which is
        a different design from the one written here and needs deciding before
        it is built. Three further reasons to leave it:

        - it changes how every XRCE fixture builds, so it costs a tier-2 sweep
          to believe, and steps 1-2 are worth landing on their own;
        - the two compiles are not yet equivalent and nothing says which is the
          contract: the cargo lane builds for six target families at `-Os` with
          `-ffunction-sections` and `UCLIENT_PLATFORM_NO_POSIX`, the CMake lane
          is POSIX-only at `-Wall -Wextra -Wpedantic`;
        - the measurement above shrinks the prize. Since the CMake lane reaches
          no image, one compile buys correctness for nothing that ships; its
          value is that the CTest harness compiles what images compile, which
          is real and much smaller than "the risky one" implies.

        **Also found, not fixed (pre-existing).** The five XRCE pool knobs that
        size ~86 % of `xrce_session_state_t` — `NROS_XRCE_MAX_SUBSCRIBERS`,
        `MAX_SERVICE_SERVERS`, `MAX_SERVICE_CLIENTS`, `SUBSCRIBER_RING_DEPTH`,
        `BUFFER_SIZE` — appear nowhere in
        `book/src/reference/static-pool-inventory.md`, before this change or
        after. That is issue 0271's own rule ("a knob nobody can enumerate is a
        knob nobody sets") going unmet, and it is now cheap to fix, because
        `xrce-config.txt` enumerates exactly those five with their minimums;
        `gen-pool-inventory.py` would have to read it.

        **The zenoh invariant does NOT transfer here.** `check-zenoh-source-
        manifest` can say "only `src/system/<platform>/` paths may be
        conditional" because zenoh-pico's per-platform axis is visible in the
        PATH. XRCE's `posix` / `posix_ip` groups hold ordinary `uxr`/`backend`
        paths with no directory-shaped tell — the attachment fact there is
        "which files need a POSIX libc", which a path cannot answer. Whatever
        gates the XRCE vendor build needs a different predicate; do not copy
        the zenoh one over and assume it holds.
      - zenoh: fold `zephyr/cmake/nros_rmw_zenoh.cmake`'s `GLOB_RECURSE` into the
        same source list `nros-zpico-build` computes. **LANDED 2026-09-05.**
        `packages/rmw/zenoh/zpico-sys/zenoh-sources.txt` is the one list; neither
        lane names a vendored path any more. It names **DIRECTORIES**, not files,
        and that is the one place it departs from `xrce-sources.txt`
        deliberately: the XRCE list is a deliberate SUBSET of a much larger tree,
        so each line carries a decision, while zenoh-pico's selection is a RULE
        ("the whole core, nine subtrees, recursively") over a REBASED PATCH LINE
        that moves with upstream — expanding it to ~130 paths would restate a
        rule as data, freeze upstream's layout in 130 places, and tax every
        submodule bump with a reconciliation whose only correct answer is "add
        them all". The safety a file list buys is bought instead by the gate:
        `check-zenoh-source-manifest` asserts every `.c` in the tree is covered
        by a record, left to the per-platform axis, or on a documented
        not-compiled list, so a new file in a listed directory is compiled (which
        is what was wanted) while a new DIRECTORY fails. The Zephyr divergence —
        `system/zephyr/network.c` from the tree rather than the alias TU (phase
        160.C's ABI mismatch), plus `isotp.c` behind
        `CONFIG_NROS_ZENOH_LINK_ISOTP` — is carried as the named conditions
        `zephyr` / `zephyr_isotp`, which the cargo lane answers with
        `platform == "zephyr"` (false by construction: Zephyr is `compiled_by =
        "platform"`, issue 0541) rather than a literal `false`.
        **The gate needed one check that is not about SHAPE, and the first
        version did not have it.** Six mutations each broke the manifest's
        structure — an undeclared group, a dropped directory, a regrown glob, an
        unanswered token, a dead token, a regrown path — and every one was
        caught; the seventh moved a record between two legitimately-declared
        groups and was not. `dir core … utils` → `dir zephyr_system … utils`
        leaves the group count, the record count, the condition-token set and
        the tree coverage all exactly right, and drops nine `.c` from every
        non-Zephyr cargo build, quietly enough that the reader's "selected no
        sources" guard does not fire because 125 is not 0. Same hole the XRCE
        version gate had next door: a gate that checks shape does not check that
        each record is attached to the right thing. The invariant added:
        **platform-conditional compilation is legitimate only for the platform
        trees** — a path under `src/system/<platform>/` must be in a CONDITIONAL
        group and everything else, being core, in an unconditional one, so
        `path is per-platform ⟺ condition != always`. It is this manifest's own
        argument (the core is a rule; the only per-file decisions are the
        platform trees) turned into a check, and it covers the mirror direction
        for free: a `system/zephyr/*.c` in `core` would compile a Zephyr-only TU
        on every platform.
        Measured, not asserted: `cargo build -p zpico-sys` compiles the same 134
        `.c` before and after (compiler-wrapper capture, empty diff), and the
        cmake reader yields the same 133 / 134 paths the old globs did, with and
        without ISO-TP. **Found while doing it:** the old `GLOB_RECURSE` carried
        no `CONFIGURE_DEPENDS`, so a `.c` appearing under one of those
        directories — which is exactly what an upstream rebase does — had NO
        rebuild edge; the new `dir` expansion passes it.
        Still outside the manifest, on purpose: the OTHER platforms'
        `src/system/<platform>/*.c`, declared once each in
        `packages/platform/nros-platform-*/nros-platform.toml` `extra_sources`
        (one declaration, one lane — no mirror to drift), and the in-repo TUs
        (`zpico.c`, `zpico_zephyr.c`, `platform_aliases.c`, `size_probe.c`),
        which are not part of the vendored tree.
      Both are build-affecting and need the submodules checked out and both lanes
      built; neither is a documentation change, and neither should be attempted
      in the same commit as the identity above.

## Risks

- **W3 touched 371 files** (the estimate said ~170; the gap is the 73 plain
  `cmake`/`cargo` workspace members and the 34 packages that declared nothing).
  Mechanical, but it is exactly the kind of sweep that hides one semantic
  change. Kept the rewrite and any behavioural change in separate commits.
- **W9 moves pins.** The cyclonedds and zenoh-pico pins are decisions, not lags;
  the wave must not become an excuse to bump them.
- **W4 changes what a stock colcon does with our packages.** That is the intent,
  but it will look like a regression to anyone who was relying on the accident.

## Out of scope

Install prefixes and a sourced `setup.sh`; per-package isolated builds for
in-tree packages; a Python plugin ABI; rosdep. RFC-0087 D8 records why for each.


## Adopted issue (2026-09-04)

* **[#1054](../issues/archived/1054-provider-scan-prunes-the-nano-ros-root.md)** —
  `provider_scan` reads `.nros-ignore` on the root it was handed, so scanning the
  nano-ros tree finds nothing. The marker's own header (issue 0621) says it
  prunes a tree from any walk that starts ABOVE it; honouring it at the root
  inverts that. Provider discovery is this phase's subject, and a scan that
  returns an empty set silently is the worst shape it can take.
