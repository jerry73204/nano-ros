# nano-ros gates: abi
#
# The C ABI and its Rust mirror -- bindgen output, cbindgen headers,
# vtable shape, and the hand-mirrored structs that drift on append.
#
# One of the topic files `just/check.just` imports. The names stay FLAT -- a
# gate is still `just check <name>` -- because `import` merges definitions
# rather than namespacing them, and ~350 references across docs, scripts and
# workflows spell them that way.
#
# Adding a gate here does not touch any other file, which is the point: for
# most of this repo's life every new gate appended to ONE file, and
# `just/check.just` became the worst conflict surface in the tree at 48 of
# 300 commits.

abi-bindings:
    #!/usr/bin/env bash
    set -euo pipefail
    if ! command -v bindgen >/dev/null 2>&1; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip abi-bindings \
            "bindgen-cli not installed (cargo install bindgen-cli --locked --version 0.72.1)"
        exit 0
    fi
    bash scripts/gen-abi-bindings.sh >/dev/null
    if ! git diff --exit-code --quiet -- \
        packages/rmw/cffi/src/generated.rs \
        packages/platform/nros-platform-cffi/src/generated.rs \
        packages/boards/nros-board-cffi/src/generated.rs; then
        git --no-pager diff --stat -- \
            packages/rmw/cffi/src/generated.rs \
            packages/platform/nros-platform-cffi/src/generated.rs \
            packages/boards/nros-board-cffi/src/generated.rs
        echo "ERROR: committed ABI bindings are stale — headers changed without rerunning scripts/gen-abi-bindings.sh; commit the regenerated files."
        exit 1
    fi
    echo "ABI bindings match the C-header SSoT."

# Phase 176.4 — verify <nros/board.h> matches the Rust extern block
# and the `nros_board_export!` macro emission in nros-board-cffi.
[private]
board-abi-mirror:
    @bash scripts/check-board-abi-mirror.sh

# Issue 0452 — the committed cbindgen headers must match a fresh generation.
# The Rust->C mirror of `check-abi-bindings`, which has guarded the C->Rust
# direction since RFC-0054. Builds no longer regenerate these in place, so
# without this gate a stale header could sit in the tree indefinitely.
[private]
cbindgen-headers:
    @cargo run -q -p nros-cbindgen-headers -- --check

# Issue 0452 — cbindgen's requirement must stay EXACT and inherited, because the
# headers it writes are COMMITTED and it rewrites them in place on every build.
# The root lock does not govern the leaves that regenerate them (an example's
# lock is never committed), so a caret req is how 0.29.4 output lands in a tree
# whose lock says 0.29.3. Buildless.
[private]
cbindgen-pin:
    @bash scripts/check-cbindgen-pin.sh

# Issue 0586 — the C++ FFI must not discard a backend error in favour of
# `-100 TRANSPORT_ERROR`, the code its own source documents as the catch-all for
# UNMAPPED variants. Also holds the two mappers exhaustive, so a NEW variant
# fails to compile until someone maps it (issue 0557 is what the collapse cost).
[private]
cpp-ffi-error-mapping:
    @python3 scripts/check-cpp-ffi-error-mapping.py

# Issue 0160 — hand-mirrored FFI structs (component.h vs cbindgen's
# nros_cpp_ffi.h) must not drift on append (the #131 stale-mirror ABI class).
[private]
ffi-struct-mirrors:
    @bash scripts/check-ffi-struct-mirrors.sh

# Issue 0454 / phase-354 W3 — an FFI taking `goal_cdr` must strip the
# encapsulation header before handing bytes to `send_goal_raw`, which takes
# FIELDS and appends them after a header the writer already wrote.
goal-cdr-stripped:
    @python3 scripts/check-goal-cdr-stripped.py

# Issue 0472 — every `*_OPAQUE_U64S` macro must have a compile-time size guard,
# so a probe that under-states a size is a build error and not a short buffer
# written past in C.
opaque-storage-guards:
    @python3 scripts/check-opaque-storage-guards.py

# Phase 121.4.b — verify <nros/platform.h> matches the Rust extern block
# and the `nros_platform_export_*!` macro emissions in nros-platform-cffi.
[private]
platform-abi-mirror:
    @bash scripts/check-platform-abi-mirror.sh

# Issue 1126 — a doc may not name a return code neither ABI defines.
#
# `nros_ret_t` and `nros_rmw_ret_t` share a prefix family and not a vocabulary,
# in BOTH directions: `BUFFER_TOO_SMALL` is RMW-only, `TRY_AGAIN` user-only.
# Both headers are in scope in the same TU, so a comment naming the wrong half
# reads as fact and compiles to nothing — the caller who copies it gets
# "undeclared identifier" with nothing pointing back at the doc. The issue
# reported one site and argued against a gate from a ONE-SPELLING grep; the
# general question found five, including one in `rmw_vtable.h`, the header a
# third-party backend author writes against. Buildless, ~0.2 s, fast line.
ret-code-citations:
    @python3 scripts/check-ret-code-citations.py

# Phase 376 W5 — the campaign's claim, re-proven per commit.
#
# `check-rmw-api-parity` asks "is every contract symbol CLASSIFIED?"; this asks
# "does the vtable actually MIRROR upstream?" — name, args and return type per
# slot, with every difference declared. Both read committed snapshots
# (`docs/reference/rmw-implementation-{contract,signatures}.txt`), so neither
# needs a ROS install and both run on the fast line.
rmw-abi-shape:
    @python3 scripts/rmw-abi-shape.py --self-test
    @python3 scripts/rmw-abi-shape.py --check

# `book/src/reference/rmw-api-comparison.md` is GENERATED from the ROS 2
# signature extract, the nano-ros ABI headers, and the same reason map
# `rmw-api-parity` enforces — so the book cannot drift from the headers. A
# hand-maintained comparison describes the tree on the day it was written;
# phase-379's own doc claimed a missing shutdown hook months after it shipped.
[private]
rmw-api-comparison:
    @python3 scripts/gen-rmw-api-comparison.py --check

# Phase 376 — every symbol in the rmw implementation contract is classified:
# a vtable slot, another layer, or a declined RTOS reason. Reads the RECORDED
# contract, so it needs no ROS install; regenerate that with `--contract` in the
# distrobox when the distro moves.
[group("check")]
rmw-api-parity:
    @python3 scripts/rmw-api-parity.py --self-test
    @python3 scripts/rmw-api-parity.py --check

# phase-340 W2 — the preconditions for putting a platform's rows in a SHARED
# cargo target dir. A group writes into one flat `<profile>/` namespace, and
# cargo does not hash the final artifact name the way it hashes `deps/`, so two
# rows in one group producing a binary of the same name overwrite each other
# and one test silently runs the other's binary. Nothing checked this: the one
# migrated platform (qemu-arm-baremetal) is collision-free by luck, and `linux`
# — the next candidate — is not.
#
# `check-fast`: buildless and source-free (it reads examples/fixtures.toml and
# the tracked leaf Cargo.tomls, invokes no cargo and no rustc), so it runs green
# on the pristine per-push checkout the tier is measured against.
#
# phase-343 W3 — the gate itself is tripwired. It has been widened twice (W1's
# row-keyed owners, B1's unhashed LIB artifacts) and both widenings were verified
# by hand into a commit message, which is not something a later reader can
# re-run. `fixture_group_collision_gate.sh` collides a binary name and a
# staticlib name on real leaf manifests and asserts the gate REPORTS each by
# name — matching the message, not just the exit code, because a malformed
# perturbation exits non-zero too (the first draft's T2 passed with B1 reverted,
# on a TOMLDecodeError). Each arm was confirmed to fail when its half of the gate
# is disabled.
# phase-347 W2 — the RMW descriptors (RFC-0071) agree with the live lowering.
#
# W2 adds `nros-rmw.toml` per backend and changes nothing else, so a descriptor
# is a SECOND derivation of what `resolve_rmw()` already computes. This gate is
# what makes it one: while both exist they must agree, and W3 may only delete
# the closed lists once this has been green. Buildless — it reads the generated
# `NanoRosRmwDispatch.cmake`, which is itself gated for staleness against the
# resolver, so agreeing with it IS agreeing with the resolver.
[private]
rmw-descriptors:
    @python3 scripts/check-rmw-descriptors.py

# Issue 0773 — the length-or-status shape, gated.
#
# This script was a REPORT (`just rmw-ret-sign`), on no lane, because its job
# was a one-off migration sweep before W3.d step B flipped the return values.
# The flip then broke a class it could not see — backend-internal helpers
# multiplexing a byte count with an `NROS_RMW_RET_*` code — and nobody was
# running it anyway. It has a structural check now and a non-zero exit, so it
# belongs on the line.
rmw-ret-sign:
    @python3 scripts/check-rmw-ret-sign.py

# issue 0800 — "the slot exists" was reading as "the capability works": 42 slots
# had no producer, mixing optional-by-design with real gaps so no reader could
# tell them apart. This splits by producer AND consumer. A slot nothing writes
# and nothing reads is legitimate — an ABI mirroring upstream reserves the shape
# first — but it must be a DECISION, so it belongs to a declared family. Source
# -only, no ROS install, fast line.
rmw-slot-producers:
    @python3 scripts/check-rmw-slot-producers.py --self-test
    @python3 scripts/check-rmw-slot-producers.py --check

# phase-320 W2 — board support tiers must match the evidence, and every board
# package must be enumerated. A tier that is merely asserted drifts: the book
# claimed ARM FVP was "Tested" (legend: "boots in CI") for a license-walled
# target, and matrix.rs carried FVP `Runtime` cells whose tests always skip.
# Also checks the generated support table is not stale. Buildless.
# Issue 0833 — RFC-0035's slot table is GENERATED from `rmw_vtable.h` plus
# `first_missing_vtable_slot`'s `require!()` list, the same source
# `check-rmw-required-slots` reads, so the table and that gate cannot disagree.
# It had drifted to describing a 33-slot ABI that was really 74: 17 dead names,
# 61 slots unmentioned. Buildless.
[private]
rmw-slot-table:
    @python3 scripts/gen-rmw-slot-table.py --self-test
    @python3 scripts/gen-rmw-slot-table.py --check

# Issue 0780 — a POSITIONAL vtable initialiser must agree with the header.
#
# Two backends fill `nros_rmw_vtable_t`'s 70-odd slots positionally, annotated
# only by `/*slot*/` comments that nothing checks. Insert a slot mid-header and
# every line below initialises its neighbour while still carrying the old name.
# The compiler catches that only when the shifted types disagree — which is
# luck: adjacent slots sharing a signature swap silently.
rmw-vtable-order:
    @python3 scripts/check-vtable-positional-order.py

# phase-421 W4 / RFC-0088 — the `serdes` provider descriptors. Catches: an
# `impl` that is not "schema"/"codegen"; a `format_id` that is missing, not an
# integer, or outside 1..=255; an unknown key under `[serdes]`, which lowers to
# nothing while reading like a decision; two providers claiming one name or —
# the wire-visible one — one `format_id`; a descriptor whose reserved
# discriminant disagrees with `nros_serdes::format::SerializationFormatId`
# (`cdr = 1`, `uorb = 2` are written in both places, and this is what stops them
# diverging), in EITHER direction; and a package announcing
# `<nano_ros_provides kind="serdes"/>` with no descriptor beside it, whose
# symptom is otherwise a defaulted `impl` nobody chose. Runs its own selftest on
# the normal path. Buildless: TOML plus a regex over one Rust file.
[private]
serdes-descriptors:
    @python3 scripts/check-serdes-descriptors.py

# issue 0734 — a binary links exactly ONE nano-ros Rust staticlib. A staticlib
# bundles its whole dependency closure, so linking two duplicates it — and
# because they are separate cargo builds with different `-C metadata`, the
# duplicated statics do not collide and BOTH get allocated. Sibling if/else arms
# are fine and expected; the gate only flags two umbrellas on ONE branch.
single-rust-staticlib:
    @python3 scripts/check-single-rust-staticlib.py

# Phase 247 W2 (issue 0050) — fast source-level weak-symbol gate: fail when an
# owned C/C++/asm file outside the audited allowlist
# (scripts/weak-symbols-allowlist.txt, shared with weak_symbol_audit.rs)
# introduces a weak symbol, or a listed file's count drifts. Buildless + fast.
# The deeper image gate is `just check weak-symbols-image` (needs fixtures).
[private]
weak-symbols:
    @bash scripts/check-weak-symbols.sh

# Phase 247 W1 (issue 0050) — image-level weak-symbol gate: assert each
# override-default weak symbol is STRONG-overridden in the final linked images
# (firmware ELFs / executables), not silently left weak. Needs prebuilt
# fixtures (skips covered classes whose artifacts are absent) — NOT in the fast
# `check` aggregate; run after the fixture build / in the per-platform CI lanes.
# The fast source-level half is `weak_symbol_audit.rs` (in `just test`).
weak-symbols-image:
    @bash scripts/check-weak-symbols-image.sh
