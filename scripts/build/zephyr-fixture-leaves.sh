#!/usr/bin/env bash
# Emit Zephyr fixture leaf records without building them.
#
# Prototype for Phase 226 fixture scheduling: keep just/zephyr-ci.just as the
# build owner for now, but centralize the matrix identity and derived settings.
set -euo pipefail

usage() {
    cat >&2 <<'EOF'
usage: scripts/build/zephyr-fixture-leaves.sh --emit records [OPTIONS]

Emits one tab-separated Zephyr fixture leaf record per selected matrix row.
This diagnostic mode does not run west, ninja, cargo, cmake, or just.

Options:
  --emit records            required; emit fixture leaf records
  --zephyr-version VERSION  Zephyr line selector (default: $NROS_ZEPHYR_VERSION or 3.7)
  --nros-root DIR           nano-ros checkout root (default: current repo)
  --build-root DIR          Zephyr build root (default: $NROS_ZEPHYR_BUILD_ROOT,
                            else selected workspace path)
  --codegen-tool PATH       host codegen tool used in signatures
                            (default: resolved nros CLI, or build/host-codegen/nros-codegen
                            when nros is unavailable)
  --make-bin PATH           make path used in signatures (default: $(nros sdk-path make)/bin/make
                            when executable, else command -v make)
  --toolchain-cache-dir DIR Zephyr toolchain capability cache dir
                            (default: build/zephyr-cache/ToolchainCapabilityDatabase)
  --sccache-disable 0|1     match NROS_ZEPHYR_SCCACHE_DISABLE (default: env or 0)
  --pristine auto|always|never
                            record desired pristine mode (default: env or auto)
  --filter REGEX            filter against "board build_dir src conf_files id"
                            (default: $NROS_ZEPHYR_FIXTURE_FILTER)
  --include-workspace-entry also emit the Zephyr workspace-Entry leaf
  -h, --help                show this help

Record fields:
  kind id target board lang lang_tag role rmw src src_dir build_name build_dir
  log xrce_agent_port zenoh_locator cyclone_domain conf_files extra_cmake_defs
  sig sig_file best_effort eff_pristine ws_dir nros_image
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
default_nros_root="$(cd "$script_dir/../.." && pwd)"

emit=""
zephyr_version="${NROS_ZEPHYR_VERSION:-3.7}"
nros_root="$default_nros_root"
build_root=""
codegen_tool=""
make_bin=""
toolchain_cache_dir=""
sccache_disable="${NROS_ZEPHYR_SCCACHE_DISABLE:-0}"
pristine="${NROS_ZEPHYR_PRISTINE:-auto}"
fixture_filter="${NROS_ZEPHYR_FIXTURE_FILTER:-}"
include_workspace_entry=0

while [ "$#" -gt 0 ]; do
    case "$1" in
        --emit)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            emit="$1"
            ;;
        --zephyr-version)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            zephyr_version="$1"
            ;;
        --nros-root)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            nros_root="$1"
            ;;
        --build-root)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            build_root="$1"
            ;;
        --codegen-tool)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            codegen_tool="$1"
            ;;
        --make-bin)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            make_bin="$1"
            ;;
        --toolchain-cache-dir)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            toolchain_cache_dir="$1"
            ;;
        --sccache-disable)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            sccache_disable="$1"
            ;;
        --pristine)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            pristine="$1"
            ;;
        --filter)
            shift
            [ "$#" -gt 0 ] || { usage; exit 2; }
            fixture_filter="$1"
            ;;
        --include-workspace-entry)
            include_workspace_entry=1
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage
            exit 2
            ;;
    esac
    shift
done

if [ "$emit" != "records" ]; then
    usage
    exit 2
fi

case "$zephyr_version" in
    3.7|4.4) ;;
    *)
        echo "zephyr-fixture-leaves: unsupported --zephyr-version=$zephyr_version" >&2
        exit 2
        ;;
esac
case "$sccache_disable" in
    0|1) ;;
    *)
        echo "zephyr-fixture-leaves: invalid --sccache-disable=$sccache_disable; expected 0 or 1" >&2
        exit 2
        ;;
esac
case "$pristine" in
    auto|always|never) ;;
    *)
        echo "zephyr-fixture-leaves: invalid --pristine=$pristine; expected auto, always, or never" >&2
        exit 2
        ;;
esac

nros_root="$(cd "$nros_root" && pwd)"
# shellcheck source=scripts/build/fixture-matrix.sh
source "$nros_root/scripts/build/fixture-matrix.sh"
# shellcheck source=scripts/build/cargo.sh
source "$nros_root/scripts/build/cargo.sh"

if [ -z "$build_root" ]; then
    if [ -n "${NROS_ZEPHYR_BUILD_ROOT:-}" ]; then
        build_root="$NROS_ZEPHYR_BUILD_ROOT"
    elif [ "$zephyr_version" = "4.4" ]; then
        build_root="$nros_root/../nano-ros-workspace-4.4"
    elif [ -d "$nros_root/zephyr-workspace" ]; then
        build_root="$nros_root/zephyr-workspace"
    elif [ -d "$nros_root/../nano-ros-workspace" ]; then
        build_root="$nros_root/../nano-ros-workspace"
    else
        build_root="$nros_root/zephyr-workspace"
    fi
fi
if [ -z "$codegen_tool" ]; then
    codegen_tool="$(nros_cargo_codegen_c_bin 2>/dev/null || true)"
    if [ -z "$codegen_tool" ]; then
        codegen_tool="$nros_root/build/host-codegen/nros-codegen"
    fi
fi
if [ -z "$toolchain_cache_dir" ]; then
    toolchain_cache_dir="$nros_root/build/zephyr-cache/ToolchainCapabilityDatabase"
fi
if [ -z "$make_bin" ]; then
    # `nros` is NOT on PATH in every lane that emits records, and this mode is a
    # pure emitter that never runs make -- it only names it in the signature.
    # `gate.yml` builds the CLI only on pull_request/merge_group/schedule/
    # dispatch, so on a `push` there is no `nros`, and an unguarded call exits
    # 127 under `set -e` and takes the whole gate with it. That is what
    # `check-fixtures-manifest` and `check-kconfig-overridden-values` did on
    # EVERY push to main, from 2026-08-31 (321642a20, which introduced this
    # resolution) until 2026-09-05 -- a uniformly red lane, which is the one
    # state that cannot report a regression.
    #
    # The guarded shape is not new: `check-tier-preconditions.sh` resolves the
    # same tool the same way and already carries the `command -v` test. This is
    # its unguarded sibling.
    _sdk_make=""
    if command -v nros >/dev/null 2>&1; then
        _sdk_make="$(nros sdk-path make 2>/dev/null || true)"
    fi
    if [ -n "$_sdk_make" ] && [ -x "$_sdk_make/bin/make" ]; then
        make_bin="$_sdk_make/bin/make"
    else
        make_bin="$(command -v make || true)"
    fi
    unset _sdk_make
fi

log_dir="$nros_root/build/zephyr-fixtures"
build_root="$(realpath -m "$build_root")"
codegen_tool="$(realpath -m "$codegen_tool")"
toolchain_cache_dir="$(realpath -m "$toolchain_cache_dir")"
if [ -n "$make_bin" ]; then
    make_bin="$(realpath -m "$make_bin")"
fi

if [ "$zephyr_version" = "4.4" ]; then
    native_sim_nsos_conf="$nros_root/cmake/zephyr/native-sim-line-4.4.conf"
else
    native_sim_nsos_conf="$nros_root/zephyr/native-sim-nsos.conf"
fi

fixture_rmws=(zenoh)
if [ "$zephyr_version" != "4.4" ]; then
    fixture_rmws=(zenoh xrce)
    nros_store_root="${NROS_HOME:-$HOME/.nros}/sdk"
    if command -v idlc >/dev/null 2>&1 \
        || [ -n "$(ls "$nros_store_root"/cyclonedds/*/bin/idlc 2>/dev/null)" ] \
        || [ -x "$nros_root/build/cyclonedds/bin/idlc" ] \
        || [ -x "$nros_root/build/install/bin/idlc" ]; then
        fixture_rmws+=(cyclonedds)
    fi
fi

escape_field() {
    local value="$1"
    value="${value//$'\t'/\\t}"
    value="${value//$'\n'/\\n}"
    printf '%s' "$value"
}

emit_record() {
    local kind="$1"
    local id="$2"
    local target="$3"
    local board="$4"
    local lang="$5"
    local lang_tag="$6"
    local role="$7"
    local rmw="$8"
    local src="$9"
    local src_dir="${10}"
    local build_name="${11}"
    local build_dir="${12}"
    local log="${13}"
    local xrce_agent_port="${14}"
    local zenoh_locator="${15}"
    local cyclone_domain="${16}"
    local conf_files="${17}"
    local extra_cmake_defs="${18}"
    local sig="${19}"
    local sig_file="${20}"
    local best_effort="${21}"
    local eff_pristine="${22}"
    # phase-383 W9.b — the retarget pair. EMPTY on an unmigrated leaf, which is
    # how the runner tells `west build` from `nros build <bringup>:<image>`.
    local ws_dir="${23:-}"
    local nros_image="${24:-}"

    local fields=(
        "$kind" "$id" "$target" "$board" "$lang" "$lang_tag" "$role" "$rmw"
        "$src" "$src_dir" "$build_name" "$build_dir" "$log" "$xrce_agent_port"
        "$zenoh_locator" "$cyclone_domain" "$conf_files" "$extra_cmake_defs"
        "$sig" "$sig_file" "$best_effort" "$eff_pristine" "$ws_dir" "$nros_image"
    )
    local i
    for i in "${!fields[@]}"; do
        if [ "$i" -gt 0 ]; then
            printf '\t'
        fi
        escape_field "${fields[$i]}"
    done
    printf '\n'
}

variant_offset_for_role() {
    case "$1" in
        talker|listener) printf '%s\n' 0 ;;
        service-server|service-client) printf '%s\n' 10 ;;
        action-server|action-client) printf '%s\n' 20 ;;
        *) echo "unknown Zephyr fixture role: $1" >&2; return 2 ;;
    esac
}

variant_idx_for_role() {
    case "$1" in
        talker|listener) printf '%s\n' 0 ;;
        service-server|service-client) printf '%s\n' 1 ;;
        action-server|action-client) printf '%s\n' 2 ;;
        *) echo "unknown Zephyr fixture role: $1" >&2; return 2 ;;
    esac
}

lang_offset_for_lang() {
    case "$1" in
        rust) printf '%s\n' 0 ;;
        c) printf '%s\n' 100 ;;
        cpp) printf '%s\n' 200 ;;
        *) echo "unknown Zephyr fixture language: $1" >&2; return 2 ;;
    esac
}

lang_idx_for_lang() {
    case "$1" in
        rust) printf '%s\n' 0 ;;
        c) printf '%s\n' 1 ;;
        cpp) printf '%s\n' 2 ;;
        *) echo "unknown Zephyr fixture language: $1" >&2; return 2 ;;
    esac
}

# phase-350 W1 — the leaf table is `examples/fixtures.toml`, read through
# `fixtures-manifest.py west-leaves`.
#
# This used to be a nested `lang × rmw × role` loop over `nros_fixture_langs` /
# `nros_fixture_roles` / `fixture_rmws`, plus a separate block for the mps2
# witness leaves and another for logging-smoke. That was a SECOND spelling of a
# matrix the manifest is supposed to own (issue 0535): the leaves had no
# `row_coord()`, so no lane could select a coordinate inside the zephyr module
# and the run side had to treat every artifact as unattributable.
#
# What stayed here, deliberately: the ISOLATION FORMULA. A role leaf's zenoh
# port / xrce port / cyclone domain is `alloc::port_of(...)` arithmetic over
# (lang, role), mirrored in `nros_tests::alloc` — exporting the computed value
# from the manifest would make the manifest a second spelling of the ALLOCATOR,
# trading one duplication for another. So the row carries identity and this
# script keeps the formula, except where the row authored a literal the formula
# cannot produce (the mps2 witness leaves sit on a different board's slots at
# the SLIRP host address, and logging-smoke has none at all).
# phase-350 W1.b — `NROS_FIXTURE_COORDS` narrows this lane to one CI lane's
# coordinates, the same env var and the same `--coords-from` filter every other
# fixture builder already goes through (`fixtures-build.sh`,
# `workspace-fixtures-build.sh`). Read from the env for the same reason they do:
# the callers are `just` recipes with their own positional args.
#
# Before the rows existed this lane had no coordinate to filter ON, so it could
# only be taken or left wholesale — which is why `lane=tier2`, needing two
# zephyr coordinates, built all 70 leaves and serial-added ~40 min to every
# sweep (issue 0509). Measured after: tier2 selects 7 of 70, nightly 38 of 70.
#
# An empty-or-absent file is FATAL, not a silent fallthrough: falling through
# would build everything while the log says "lane", which is the failure mode
# issue 0482 is about.
coords_args=()
if [ -n "${NROS_FIXTURE_COORDS:-}" ]; then
    if [ ! -s "${NROS_FIXTURE_COORDS}" ]; then
        echo "zephyr-fixture-leaves: NROS_FIXTURE_COORDS=${NROS_FIXTURE_COORDS} is empty or absent" >&2
        echo "                       (a silent fallthrough would emit every leaf and look like a lane)" >&2
        exit 2
    fi
    coords_args=(--coords-from "$NROS_FIXTURE_COORDS")
fi

selected=0
# `_row_coord` is read but unused HERE: this script's narrowing already happened
# in `--coords-from` above, on the same `row_coord`. It is named so the trailing
# field cannot be absorbed by `row_nros_image` — `read` puts the remainder in the
# last variable, so an unnamed column silently corrupts the one before it.
while IFS=$'\x1f' read -r board lang lang_tag rmw role src build_name id \
    row_zenoh_locator row_xrce_port row_cyclone_domain row_conf_files _row_reserved \
    row_ws_dir row_nros_image _row_coord; do
    [ -n "$board" ] || continue

    # Host gating, unchanged: cyclonedds leaves need an idlc. The manifest lists
    # them unconditionally (a row is an identity, not a host capability), so the
    # skip moves here — which is also what lets `check-zephyr-fixture-rows.py`
    # compare the two sides per-RMW instead of demanding raw set equality.
    if [ "$rmw" = "cyclonedds" ] && ! printf '%s\n' "${fixture_rmws[@]}" | grep -qx cyclonedds; then
        continue
    fi

    # issue 0549 — logging-smoke is an ORDINARY leaf now. It used to be gated
    # behind `--include-logging-smoke`, a flag no build lane ever passed, so the
    # leaf was emitted for inventory and never produced while a separate
    # `just zephyr build-logging-smoke` recipe built the image the test reads.
    # One builder, one dir, built with the lane.
    if [ "$role" = "entry" ] && [ "$include_workspace_entry" != "1" ]; then
        continue
    fi

    build_dir="$build_root/$build_name"
    # `src` is repo-relative in the manifest; the example leaves all sit under
    # `examples/`, and logging-smoke does not.
    case "$src" in
        examples/*) src_rel="${src#examples/}"; src_dir="$nros_root/$src" ;;
        *) src_rel="$src"; src_dir="$nros_root/$src" ;;
    esac
    # phase-383 W9.b — `nros build` is addressed from the WORKSPACE, which is
    # not `src_dir` (that is the application). Repo-relative in the record, so
    # absolute here, like every other path this script hands the runner.
    ws_dir=""
    [ -z "$row_ws_dir" ] || ws_dir="$nros_root/$row_ws_dir"
    best_effort=0
    xrce_agent_port=""
    zenoh_locator=""
    cyclone_domain=""
    extra_cmake_defs="-D_NANO_ROS_CODEGEN_TOOL=$codegen_tool -DZEPHYR_TOOLCHAIN_CAPABILITY_CACHE_DIR=$toolchain_cache_dir -DMAKE=$make_bin -DUSE_CCACHE=0"


    # Conf overlays: the row carries the RELATIVE list, this script appends the
    # absolute board/NSOS tail (a host path, not a fact about the fixture).
    conf_files=""
    if [ -n "$row_conf_files" ]; then
        # issue 0260 — the default used to be the native_sim NSOS overlay, i.e.
        # "any board that is not mps2 is native_sim". That held while those were
        # the only two zephyr boards, and it silently applied
        # CONFIG_NET_SOCKETS_OFFLOAD to the first board that was neither: the
        # a53 SMP fixture then took a Data Abort in
        # `socket_offload_getaddrinfo` with a NULL vtable, 12 ms in, because
        # nothing registers an offload provider on a real board.
        #
        # A real board gets NO host-simulator overlay. Listing native_sim
        # explicitly means the next new board gets nothing rather than something
        # wrong — a default that assumes is how this cost an afternoon.
        case "$board" in
            mps2_an385) conf_tail="$nros_root/cmake/zephyr/mps2-an385.conf" ;;
            native_sim*) conf_tail="$native_sim_nsos_conf" ;;
            *) conf_tail="" ;;
        esac
        if [ -n "$conf_tail" ]; then
            conf_files="$row_conf_files;$conf_tail"
        else
            conf_files="$row_conf_files"
        fi
    fi

    if [ -n "$row_zenoh_locator$row_xrce_port$row_cyclone_domain" ]; then
        # The row authored its isolation values (mps2 witness leaves).
        zenoh_locator="$row_zenoh_locator"
        xrce_agent_port="$row_xrce_port"
        cyclone_domain="$row_cyclone_domain"
    elif [ "$role" != "logging-smoke" ]; then
        # The allocator formula, for the six role leaves.
        lang_offset="$(lang_offset_for_lang "$lang")"
        lang_idx="$(lang_idx_for_lang "$lang")"
        variant_offset="$(variant_offset_for_role "$role")"
        variant_idx="$(variant_idx_for_role "$role")"
        case "$rmw" in
            xrce) xrce_agent_port=$((2400 + lang_offset + variant_offset)) ;;
            zenoh) zenoh_locator="tcp/127.0.0.1:$((7400 + lang_offset + variant_offset))" ;;
            cyclonedds) cyclone_domain=$((22 + variant_idx * 3 + lang_idx)) ;;
        esac
    fi

    [ -z "$xrce_agent_port" ] || extra_cmake_defs="$extra_cmake_defs -DCONFIG_NROS_XRCE_AGENT_PORT=$xrce_agent_port"
    [ -z "$zenoh_locator" ] || extra_cmake_defs="$extra_cmake_defs -DCONFIG_NROS_ZENOH_LOCATOR=\"$zenoh_locator\""
    [ -z "$cyclone_domain" ] || extra_cmake_defs="$extra_cmake_defs -DCONFIG_NROS_DOMAIN_ID=$cyclone_domain"
    [ -z "$conf_files" ] || extra_cmake_defs="$extra_cmake_defs -DCONF_FILE=$conf_files"

    sccache_launcher=0
    if [ "$sccache_disable" = "0" ] && command -v sccache >/dev/null 2>&1; then
        sccache_launcher=1
        extra_cmake_defs="$extra_cmake_defs -DCMAKE_C_COMPILER_LAUNCHER=sccache -DCMAKE_CXX_COMPILER_LAUNCHER=sccache"
    fi

    target="fixture/$id"
    filter_haystack="$board $build_name $src_rel $conf_files $id"
    if [ -n "$fixture_filter" ] && ! [[ "$filter_haystack" =~ $fixture_filter ]]; then
        continue
    fi
    selected=$((selected + 1))
    sig_file="$build_dir/.nros-zephyr-fixture.sig"
    sig="$(printf '%s\n' \
        "board=$board" \
        "src=$src_rel" \
        "xrce_port=$xrce_agent_port" \
        "conf_files=$conf_files" \
        "zenoh_locator=$zenoh_locator" \
        "codegen_tool=$codegen_tool" \
        "toolchain_cache_dir=$toolchain_cache_dir" \
        "make=$make_bin" \
        "sccache_launcher=$sccache_launcher")"
    # phase-383 W9.b — the retarget pair joins the signature ONLY when the row
    # is migrated. Appending an always-present (usually empty) line would change
    # the signature of all ~70 leaves at once and re-run west on every one, at
    # roughly 20 minutes each: a schema addition must not be a rebuild event for
    # rows it does not describe.
    if [ -n "$row_nros_image" ]; then
        sig="$(printf '%s\n%s\n%s' "$sig" "ws_dir=$ws_dir" "nros_image=$row_nros_image")"
    fi
    emit_record fixture "$id" "$target" "$board" "$lang" "$lang_tag" "$role" "$rmw" \
        "$src_rel" "$src_dir" "$build_name" "$build_dir" "$log_dir/${build_name}.log" \
        "$xrce_agent_port" "$zenoh_locator" "$cyclone_domain" "$conf_files" \
        "$extra_cmake_defs" "$sig" "$sig_file" "$best_effort" "$pristine" \
        "$ws_dir" "$row_nros_image"
done < <(python3 "$nros_root/scripts/build/fixtures-manifest.py" west-leaves "${coords_args[@]}")

# phase-350 W1 — the mps2 witness leaves and the logging-smoke leaf used to sit
# in two more hand-written blocks here. They are manifest rows now, emitted by
# the loop above like every other leaf: `west_build_name` carries the
# `build-cortex-m-*` naming, `west_zenoh_locator` the mps2 allocator slots
# (`alloc::port_of(ZephyrQemuCortexM, ...)` = 10600/10700/10800 at the SLIRP
# host 10.0.2.2, not 127.0.0.1), and `west_role` the logging-smoke role that is
# not a directory name. issue 0549 then retired the `--include-logging-smoke`
# flag with the duplicate builder it existed for: the leaf is ordinary now.

# Phase 225.P.6 — workspace-Entry leaf (Approach A). Constructed directly,
# bypassing variant_offset_for_role (role="entry" is unknown to it). The
# proven zephyr-fixture-run-one.sh west path builds it unchanged from the
# Zephyr application dir at examples/workspaces/rust/src/zephyr_entry. The
# Entry is the (zephyr, rust, EntryPubsub) workspace cell: since the
# phase-295 W4 re-bake it bakes the allocator's EntryPubsub slot (port
# 7430 = alloc::port_of(ZephyrNativeSim, Rust, EntryPubsub)) — DISTINCT
# from the single-node rust pubsub talker's 7400, so the two no longer
# share a router. (The e2e test dials an ephemeral router via the
# phase-286 --nros-locator override anyway; the bake is the fallback.)
# phase-350 W1 — the twelve workspace-Entry leaves used to sit here as twelve
# near-identical 48-line blocks (~570 lines). They are `[[workspace_fixture]]`
# rows now — Workspace cells, per `fixture_rows_all_modeled_by_matrix` — and
# the loop above emits them like any other leaf, gated by
# `--include-workspace-entry`. Each block's per-cell rationale moved WITH it,
# onto its row in examples/fixtures.toml, because the manifest is where the
# question "why does this cell exist?" should be answerable.

if [ "$selected" -eq 0 ]; then
    echo "zephyr-fixture-leaves: no records matched filter: $fixture_filter" >&2
    exit 1
fi
