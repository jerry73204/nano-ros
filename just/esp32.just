set working-directory := '..'

NIGHTLY := `awk '/^channel/ {gsub(/"/, "", $3); print $3; exit}' tools/rust-toolchain.toml`

default:
    @just --list --justfile {{source_file()}}

# Install ESP32 prerequisites: Espressif QEMU fork (required by every
# networked ESP32 test — the stock Debian/Ubuntu `qemu-system-riscv32`
# doesn't know the `esp32c3` machine model). Post-84.F4 zenoh-pico
# itself is cross-compiled by `zpico-sys/build.rs` during the
# example's cargo build; no standalone library build step is needed
# or wired up anywhere in the project.
#
# Phase 213.A.4 — this recipe provisions the `[board.qemu-esp32-baremetal]`
# board (Espressif QEMU fork, esp32c3 machine). It has `packages = []` in
# the index — the riscv32 target comes from `just workspace rust-targets`
# (rustup), and the QEMU fork is the source-built `[tool.esp32-qemu]`. There
# is no `just qemu-esp32-baremetal setup` sibling; this recipe is the entry point.
[group("setup")]
setup:
    #!/usr/bin/env bash
    set -e
    # Phase 197.4/196.9 — provision the build sources (zenoh-pico/mbedtls via the
    # default zenoh RMW; the riscv32 target is rustup-managed) so the cross build
    # resolves. The esp32 board has no toolchain package (build-std), so this is
    # the source set.
    source scripts/build/cargo.sh
    "$(nros_cli_bin)" setup qemu-esp32-baremetal
    # Probe for the `esp32c3` machine model, not the binary's mere existence —
    # the ESP32 e2e TESTS need Espressif's QEMU fork (stock qemu-system-riscv32
    # only knows `virt`). Build does NOT need it, so provision best-effort: the
    # fork's source-build wants qemu deps that aren't always present, and that
    # must not block the build phase (the e2e step gates on its presence).
    if command -v qemu-system-riscv32 &>/dev/null \
            && qemu-system-riscv32 -machine help 2>/dev/null | grep -q '\besp32c3\b'; then
        echo "Espressif qemu-system-riscv32 already installed: $(qemu-system-riscv32 --version | head -1)"
    else
        # Phase 197.3 — the Espressif QEMU fork is the source-built
        # [tool.esp32-qemu] in nros-sdk-index.toml; provision it via nros.
        nros setup --tool esp32-qemu || echo "esp32-qemu unavailable (e2e will skip); build unaffected"
    fi

# Diagnose ESP32 install status (read-only).
[group("setup")]
doctor:
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh esp32 doctor "just doctor esp32"
    set +e
    fail=0
    if command -v qemu-system-riscv32 &>/dev/null \
            && qemu-system-riscv32 -machine help 2>/dev/null | grep -q '\besp32c3\b'; then
        echo "  [OK] qemu-system-riscv32 (Espressif fork, esp32c3 machine)"
    else
        echo "  [MISSING] Espressif qemu-system-riscv32 with esp32c3 machine (run: just esp32 setup)"
        fail=1
    fi
    if rustup target list --installed 2>/dev/null | grep -q '^riscv32imc-unknown-none-elf$'; then
        echo "  [OK] riscv32imc-unknown-none-elf target"
    else
        echo "  [MISSING] riscv32imc-unknown-none-elf target (run: just workspace rust-targets)"
        fail=1
    fi
    if command -v espflash &>/dev/null; then
        echo "  [OK] espflash"
    else
        echo "  [MISSING] espflash (run: just workspace cargo-tools)"
        fail=1
    fi
    exit $fail

# Build ESP32 core artifacts. ESP32 has no separate core build step
# (the zenoh-pico cross-build runs inside each example's cargo build,
# Phase 84.F4) — `build` is an alias for `build-examples` so the standard
# `just <plat> build` UX returns the same artifact set as every other
# platform. Phase 208.D.5.
[group("main")]
build: build-examples

# Build ESP32 examples (requires the workspace nightly — ESP32-C3 / riscv32imc).
[group("main")]
build-examples:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    # issue 1038 — the central `nros-patch.toml` this lane's Rust leaves
    # `include`, made EXPLICIT. This lane already got the file, but only as a
    # side effect of `workspace-fixtures-build.sh` running `nros sync` somewhere
    # earlier; `threadx-riscv64` had no such neighbour and died at manifest
    # parse. An inherited artifact is a working lane, not a declared dependency.
    # Idempotent — a process spawn when the file is already current.
    nros_ensure_central_patch
    # Phase 244.D2 — the WiFi-only plain esp32 fixtures (talker/listener) were
    # dropped (WiFi un-emulable). The only `platform = "esp32"` fixture left is the
    # ESP32-C3 QEMU (OpenETH) workspace Entry, built by the workspace lane below;
    # `fixtures-build.sh esp32 rust` would now select zero plain fixtures.
    echo "Building ESP32 examples (from examples/fixtures.toml)..."
    # Phase 225.O — ESP32-C3 QEMU workspace Entry (Node + Bringup + Entry
    # pkgs) through the documented workspace workflow (`nros sync`,
    # `nros codegen-system`, then cargo). Mirrors the freertos /
    # threadx_linux build-examples workspace-lane tail. The per-row `env`
    # in examples/fixtures.toml carries RUSTUP_TOOLCHAIN +
    # CARGO_UNSTABLE_BUILD_STD, so this call needs no extra toolchain env.
    bash scripts/build/workspace-fixtures-build.sh esp32 rust
    echo "ESP32 examples built!"

# Build ESP32 QEMU examples (requires nightly)
[group("examples")]
build-qemu:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    source scripts/build/fixtures-target-dir.sh
    target_profile_dir="$(nros_cargo_target_profile_dir)"
    echo "Building ESP32 QEMU fixtures (from examples/fixtures.toml)..."
    for id in qemu-esp32-baremetal-talker qemu-esp32-baremetal-listener; do
        RUSTUP_TOOLCHAIN="{{NIGHTLY}}" SSID="${SSID:-test}" PASSWORD="${PASSWORD:-test}" \
            bash scripts/build/fixtures-build.sh qemu-esp32-baremetal rust --id "$id"
    done
    echo ""
    echo "Creating flash images..."
    # issue 0535 — the packed `.bin` is a POSTPROCESS of the cargo rows above,
    # so it has no manifest row of its own; the KIND is what this producer and
    # `esp32_emulator.rs` share instead of a literal each.
    source scripts/build/build-root.sh
    esp32_qemu_dir="$(nros_build_dir "$NROS_KIND_ESP32_QEMU")"
    mkdir -p "$esp32_qemu_dir"
    for ex in talker listener; do
        # Crate [[bin]] names use underscores (esp32_qemu_talker); the packed
        # flash image keeps the historical hyphenated name every QEMU
        # runner/test references (issue #181 — the hyphen/underscore drift
        # left espflash opening a nonexistent ELF while the lane exited 0).
        elf_name="esp32_qemu_${ex}"
        # phase-340 item 7 — ask where the build WROTE, never assume the leaf.
        # `qemu-esp32-baremetal` shares a target dir, so this row's artifacts
        # live under `build/cargo-fixtures/<group>/`; the helper returns the
        # leaf's own `target/` for any platform that has not migrated, so this
        # line is correct on both sides of a migration.
        #
        # Issue 1025 — BY ROW ID, not by leaf-plus-two-empty-strings. The group
        # key is a function of (platform, cargo args, env); this used to pass
        # `"" ""` for the last two, so the moment these rows gained
        # `env = { ZPICO_MAX_QUERYABLES = "2" }` the build wrote
        # `qemu-esp32-baremetal-4118800323` and this line kept reading
        # `qemu-esp32-baremetal`. No flash image could be packed at all. The id
        # is the same one passed to `fixtures-build.sh --id` four lines up.
        artifact_dir="$(nros_fixture_row_artifact_dir_by_id \
            "qemu-esp32-baremetal-$ex" qemu-esp32-baremetal)"
        elf="$artifact_dir/riscv32imc-unknown-none-elf/$target_profile_dir/$elf_name"
        out="$esp32_qemu_dir/esp32-qemu-$ex.bin"
        # Issue 0439 — the build loop above is lane-narrowable, so under a lane
        # that excludes `qemu-esp32-baremetal,rust` it correctly builds nothing
        # and there is no ELF to pack. Skip ONLY for that reason: an absent ELF
        # with nothing narrowing the build is issue #181 exactly (espflash
        # opening a nonexistent path while the lane exits 0), and that has to
        # stay loud. `NROS_FIXTURE_COORDS` is the same signal
        # `fixtures-build.sh` narrows on, so the two cannot disagree.
        if [ ! -f "$elf" ]; then
            if [ -n "${NROS_FIXTURE_COORDS:-}" ]; then
                echo "  skip $ex — not in this lane's coordinates"
                continue
            fi
            echo "ERROR: $elf is missing, and nothing narrowed this build." >&2
            echo "       The build step above should have produced it." >&2
            exit 1
        fi
        # Issue 0486 — a MISSING espflash is now a provisioning failure, not a
        # skip. It used to warn and continue, which was honest while nothing
        # promised the tool; `[tool.espflash]` + the board's `packages` now do,
        # so warning here would turn a broken host into a green lane with no
        # artifacts — the same shape as #181, which this recipe already refuses
        # for a missing ELF two lines up.
        if ! command -v espflash &>/dev/null; then
            echo "ERROR: espflash not found, so no flash image can be packed." >&2
            echo "       It is declared in nros-sdk-index.toml and provisioned by" >&2
            echo "         just esp32 setup      (or: nros setup --tool espflash)" >&2
            echo "       and put on PATH by \`source ./activate.sh\`." >&2
            exit 1
        fi
        espflash save-image --chip esp32c3 --flash-size 4mb --merge "$elf" "$out"
        # Issue 1065 — assert the ARTIFACT, not the exit code. This recipe's own
        # history is of exiting 0 while producing nothing (#181: espflash opening
        # a nonexistent path while the lane went green), and the guard above only
        # covers a missing INPUT. A packer that writes no output, or an empty
        # one, is the same lie one step later.
        if [ ! -s "$out" ]; then
            echo "ERROR: espflash exited 0 but $out is missing or empty." >&2
            exit 1
        fi
        echo "  $out"
    done
    echo "ESP32 QEMU examples built!"

# Phase 88.15.f — build the ESP32-C3 nros-log smoke fixture +
# espflash-pack the flash image alongside the ELF so the harness
# can pick it up via `build_logging_smoke_esp32_qemu_flash`.
[group("examples")]
build-logging-smoke:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    source scripts/build/fixtures-target-dir.sh
    target_profile_dir="$(nros_cargo_target_profile_dir)"
    echo "Building logging-smoke-esp32-qemu..."
    RUSTUP_TOOLCHAIN="{{NIGHTLY}}" SSID="${SSID:-test}" PASSWORD="${PASSWORD:-test}" \
        bash scripts/build/fixtures-build.sh qemu-esp32-baremetal rust --id qemu-esp32-baremetal-logging-smoke
    echo "Creating logging-smoke-esp32-qemu flash image..."
    fixture=packages/testing/nros-tests/bins/logging-smoke-esp32-qemu
    # Same migration-aware derivation as `build-qemu` — this row is
    # `qemu-esp32-baremetal` too, so it moved with the platform. Issue 1025:
    # by ROW ID, because this row carries `env` as well and the two-empty-string
    # form derives a group dir the build never wrote to.
    artifact_dir="$(nros_fixture_row_artifact_dir_by_id \
        qemu-esp32-baremetal-logging-smoke qemu-esp32-baremetal)"
    elf="$artifact_dir/riscv32imc-unknown-none-elf/$target_profile_dir/logging-smoke-esp32-qemu"
    bin="$elf.bin"
    # Issue 0439 — same shape as `build-qemu` above: a lane-narrowable `--id`
    # build followed by a pack that assumed it happened. Fixed here WITH the
    # reported site rather than after it bites: this recipe is only absent from
    # the tier-2 failure because its id is not in that lane's module set.
    if [ ! -f "$elf" ]; then
        if [ -n "${NROS_FIXTURE_COORDS:-}" ]; then
            echo "  skip — logging-smoke is not in this lane's coordinates"
            exit 0
        fi
        echo "ERROR: $elf is missing, and nothing narrowed this build." >&2
        exit 1
    fi
    # Issue 0486 — same as `build-qemu`: provisioned means required.
    if ! command -v espflash &>/dev/null; then
        echo "ERROR: espflash not found, so no flash image can be packed." >&2
        echo "       Provision it:  just esp32 setup   (nros setup --tool espflash)" >&2
        echo "       then re-source ./activate.sh to put it on PATH." >&2
        exit 1
    fi
    espflash save-image --chip esp32c3 --flash-size 4mb --merge "$elf" "$bin"
    echo "  $bin"

# Build ESP32 test fixtures.
[group("full-matrix")]
build-fixtures: build-qemu build-logging-smoke
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh esp32 build-fixtures "just build esp32"
    set -e
    # The ESP32-C3 QEMU workspace Entry (`esp32_entry`) that
    # `test_esp32_workspace_entry_e2e` consumes via
    # `get_prebuilt_esp32_qemu_workspace_entry`. Previously only `build-examples`
    # built it, but the test's skip message (correctly) points users to
    # `build-fixtures` — so this lane must stage it too. Idempotent + cached.
    bash scripts/build/workspace-fixtures-build.sh esp32 rust
    # Issue 1112 — the NATIVE PEERS of the esp32 interop tests.
    #
    # `test_esp32_to_native` and `test_native_to_esp32` each start one esp32
    # image and one HOST binary, so half of each test is a `linux` fixture this
    # module never builds. In the nightly that read as an esp32 failure —
    # `BuildFailed("Test fixture binary not prebuilt: .../cargo-fixtures/linux/
    # nros-relwithdebinfo/listener")`, three retries, 0.4 s each — which points
    # at the esp32 side and is not about it at all.
    #
    # Same reasoning as the workspace entry above: the tests belong to THIS
    # module, so their prerequisites are this lane's to stage. By ID rather than
    # `fixtures-build.sh linux rust`, which is 74 rows to obtain two.
    for _peer in native-rust-talker native-rust-listener; do
        bash scripts/build/fixtures-build.sh linux rust --id "$_peer"
    done
    # esp-idf build-stage fixtures (issue 0041): build the idf.py ELF the
    # cli_bringup_esp_idf test consumes, so it doesn't run idf.py at test time.
    # Self-gates on idf.py/IDF env; non-fatal (a failed build leaves no stamp,
    # and the test skips per tier). Slow (~7 min/ELF), so scoped to this esp
    # lane, not build-test-fixtures.
    # issue 0700 — NO `|| true`. It discarded the builder's exit code, so a
    # fixture that failed to build reported `ESP32 fixtures built.` and the lane
    # returned RC=0 having produced nothing. The builder now exits 0 for an
    # absent toolchain (a real skip) and non-zero only when a build was
    # attempted and produced nothing, so this can be load-bearing again.
    bash scripts/build/idf-fixtures.sh
    echo "ESP32 fixtures built."

# Build every ESP32 tier: core artifacts, examples, and test fixtures.
[group("full-matrix")]
build-all: build build-examples build-fixtures
    @echo "ESP32 build-all completed."

# Run basic QEMU ESP32-C3 boot test
[group("debug")]
test-basic: build-qemu
    #!/usr/bin/env bash
    echo "ESP32-C3 QEMU boot test"
    echo "========================"
    echo ""
    if ! command -v qemu-system-riscv32 &>/dev/null; then
        echo "WARNING: qemu-system-riscv32 not found - skipping runtime test"
        echo "Flash images are at: $(bash -c 'source scripts/build/build-root.sh; nros_build_dir "$NROS_KIND_ESP32_QEMU"')/"
        exit 0
    fi
    echo "Running boot test..."
    source scripts/build/build-root.sh
    esp32_qemu_dir="$(nros_build_dir "$NROS_KIND_ESP32_QEMU")"
    tmpfile=$(mktemp)
    trap 'rm -f "$tmpfile"' EXIT
    timeout 20 qemu-system-riscv32 -M esp32c3 -icount 3 -nographic \
        -drive "file=$esp32_qemu_dir/esp32-qemu-talker.bin,if=mtd,format=raw" \
        -nic none \
        > "$tmpfile" 2>&1 || true
    cat "$tmpfile"
    echo ""
    if grep -q "nros ESP32-C3 QEMU BSP" "$tmpfile"; then
        echo "[PASS] ESP32-C3 QEMU boot test - BSP initialized"
    else
        echo "[FAIL] ESP32-C3 QEMU boot test - BSP banner not found"
        exit 1
    fi

# Run ESP32-C3 QEMU integration tests (build, boot, E2E via nextest)
#
# Phase 214.Q: `build-fixtures` is sequenced as a prereq so the nextest
# harness never explodes with "fixture not built" errors. The harness
# fails loud on missing fixtures by design (Phase 181) — recipes own the
# build-then-test sequencing.
[group("main")]
test verbose="": build-fixtures
    @bash scripts/dev/deprecated-scope-alias.sh esp32 test "just test esp32"
    # Delegate to the shared skip-tolerant runner: the esp32_emulator tests
    # `skip!` when the Espressif QEMU fork / espflash / riscv32 target are absent
    # (best-effort emulator), and _nextest-platform treats an all-[SKIPPED] run
    # as pass while still failing on real errors.
    @just _nextest-platform esp32_emulator {{verbose}}

# Run all ESP32 tests (same as test)
[group("full-matrix")]
test-all verbose="": (test verbose)

# CI: doctor + test
[group("full-matrix")]
ci: doctor test

# Clean ESP32 build artifacts
[group("main")]
clean:
    #!/usr/bin/env bash
    for ex in talker listener; do
        rm -rf "examples/qemu-esp32-baremetal/rust/$ex/target"
    done
    source scripts/build/build-root.sh
    rm -rf "$(nros_build_dir "$NROS_KIND_ESP32_QEMU")" "$(nros_build_dir "$NROS_KIND_ESP32_ZENOH_PICO")"
    echo "ESP32 build artifacts cleaned"

# =============================================================================
# Run examples (build + launch ESP32-C3 QEMU)
# =============================================================================

# Run ESP32 talker on QEMU
talker: build-qemu
    #!/usr/bin/env bash
    set -e
    source scripts/build/build-root.sh
    qemu-system-riscv32 -M esp32c3 -icount 3 -nographic \
        -drive "file=$(nros_build_dir "$NROS_KIND_ESP32_QEMU")/esp32-qemu-talker.bin,if=mtd,format=raw" \
        -nic user,model=open_eth

# Run ESP32 listener on QEMU
listener: build-qemu
    #!/usr/bin/env bash
    set -e
    source scripts/build/build-root.sh
    qemu-system-riscv32 -M esp32c3 -icount 3 -nographic \
        -drive "file=$(nros_build_dir "$NROS_KIND_ESP32_QEMU")/esp32-qemu-listener.bin,if=mtd,format=raw" \
        -nic user,model=open_eth

# Start zenohd router for ESP32 QEMU examples (allocator rust-pubsub port 9800)
zenohd:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/zenohd.sh
    nros_router_exec tcp/127.0.0.1:9800
