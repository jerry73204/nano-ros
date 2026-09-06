#!/usr/bin/env python3
"""phase-400 — count where configuration lives, reproducibly.

The phase doc opens with a table (78 Kconfig symbols, 21 `NROS_*` env, 9
`ZPICO_*` env, against 3 knobs in the RFC-0049 ladder) and every wave's gate is
stated as "measured the same way as the table at the top". No method was
recorded, so the gate could not be evaluated: two people counting by hand get
two numbers, and a wave cannot show it moved anything.

This is that method. It is deliberately BUILDLESS — grep and TOML, no cargo —
so it can run on the fast line and so the number does not depend on a build
succeeding.

What each row counts:

  ladder knobs      fields on the typed `[knobs.*]` structs in
                    `nros-board-common`. These are the knobs that have a
                    platform and board rung and that `nros config explain`
                    reports. This is the number the phase exists to RAISE.

  Kconfig symbols   `config NROS_*` DECLARATIONS in Kconfig files — declared,
                    not referenced, because a symbol read in ten places is one
                    knob.

  build-script env  distinct `NROS_*` names a `build.rs` reads from the
                    environment. Not every `NROS_*` in the tree: a name that
                    only cmake sets and cmake reads is not a build-script knob.

  ZPICO_* env       the same, for the zenoh-pico tenant's own namespace.

A knob counted in the ladder MAY also still appear as env — that is the point:
migrating keeps the env name as the front-end. The ladder number rising is the
signal, not the env number falling to zero.
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# The ratchet. "The ladder knob count rises" is the wave gate, and a count that
# only ever rises is enforceable where a count that "should fall" is not: an
# env name legitimately SURVIVES migration as the front-end, so the env rows are
# reported, never gated. Raise this when a tenant lands.
LADDER_FLOOR = 42

# Every build-script env name, and WHAT IT IS. An unclassified name FAILS the
# gate rather than landing in a bucket by heuristic — the first version of this
# analysis split by read idiom (`env_usize` vs `env::var`) and misfiled three
# knobs that are numeric but read as strings and handed to a C `#define`.
#
#   ladder   already resolved over the RFC-0049 ladder
#   sizing   a sizing knob still to migrate — THE BACKLOG
#   derived  a number ANOTHER CAMPAIGN is making derived rather than set:
#            phase-403 / phase-408 for the receive and transmit buffers (from
#            the message type), phase-392 for the zenoh entity caps and pools
#            (from what the image declares). Migrating one of these into the
#            ladder would be WRONG: a rung gives a global a per-platform
#            default, and the point of that work is that it stops being a
#            global. The reason column names the owner.
#   infra    a path, a flag, or an input to the ladder itself. Not a knob, and
#            counting it as one overstates the backlog — which the first
#            census did, by ~40%.
KNOB_CLASS = {
    # --- ladder (phase-400 W6) ---
    # --- derived: the message type should size these ---
    "NROS_SUBSCRIPTION_BUFFER_SIZE": (
        "derived",
        "the runtime-owned take buffer. phase-403 makes it per-type; it is in "
        "the ladder today as the fallback for a type with no bound",
    ),
    "ZPICO_SUBSCRIBER_BUFFER_SIZE": ("derived", "SMALL_PAYLOADS class (phase-403)"),
    "ZPICO_SUBSCRIBER_LARGE_SIZE": ("derived", "LARGE_PAYLOADS class (phase-403)"),
    "ZPICO_SUBSCRIBER_SIZE_THRESHOLD": ("derived", "SMALL_CLASS_CEILING (phase-403)"),
    "ZPICO_PUBLISHER_TX_BUFFER_SIZE": ("derived", "TX half of the same split"),
    # --- sizing: the backlog ---
    "NROS_SERVICE_TIMEOUT_MS": ("sizing", "a timeout, not a size, but the same ladder shape. TWO readers (zenoh build + the C emitter), so it needs one emission point before a rung"),
    # issue 0968 — the UDP/TCP twin of the line above. Documented in the book and
    # in Kconfig with a 512 default, resolved by cmake under the UNPREFIXED name
    # that no build script reads, and hardcoded to 4096 on the UDP path anyway —
    # so every Zephyr XRCE image paid 2 x 4096 x 16 for its stream buffers.
    "NROS_XRCE_TRANSPORT_MTU": ("sizing", "transport MTU (UDP/TCP); numeric"),
    "ZPICO_MAX_LARGE_SUBSCRIBERS": ("derived", "pool cardinality; multiplies LARGE_PAYLOADS, phase-392"),
    "ZPICO_SERVICE_BUFFER_SIZE": ("derived", "SERVICE_BUFFERS is MAX_SESSIONS x MAX_QUERYABLES; phase-392"),
    # --- phase-392 W6: the executor backing static ---
    # NOT a ladder candidate, and the DECISION is the interesting part. A rung
    # gives a global a per-platform default, which is exactly wrong here: the
    # question this answers is "has the image's author lowered the allocator
    # arena by the same amount?", which is per-IMAGE, not per-platform (issue
    # 1145). It is also not `derived` — nothing is computing it, and claiming a
    # campaign owns it would overstate the backlog. It is the escape hatch on a
    # placement decision, so: infra.
    "NROS_EXECUTOR_BACKING_U64S": (
        "infra",
        "opt-out/size override for the `.bss` executor backing (phase-392 W6). "
        "Per-image, not per-platform: the RTOS half of the move is lowering "
        "that image's allocator arena, which only its author can measure "
        "(issue 1145)",
    ),
    "NROS_EXECUTOR_BACKING_SECTION": (
        "infra",
        "linker section name for the executor backing (phase-392 W6, amendment "
        "A / issue 0880). A placement, not a size",
    ),
    # --- infra: not knobs ---
    "NROS_ALLOW_UNRESOLVED_DEPS": ("infra", "policy flag"),
    # phase-422 W8 — its own flag, deliberately NOT reusing the one above.
    # A wrong-role dep RESOLVES; we decline it. Overloading the
    # "unresolved" hatch would make one variable mean two different
    # judgements, and a user silencing one would silence the other.
    "NROS_ALLOW_INFRA_DEPS": ("infra", "policy flag"),
    "NROS_BOOT_REPORT": ("infra", "diagnostic toggle; a bool, so it has no rung"),
    "NROS_BUILD_ROOT": ("infra", "path"),
    # Issue 1102 — regenerates the entry-codegen goldens. A test-only escape
    # hatch, not a build input: it configures nothing about an image, and the
    # goldens it rewrites are compared byte-for-byte on every other run.
    "NROS_UPDATE_GOLDEN": ("infra", "test-only golden regeneration"),
    "NROS_CARGO_FLAGS": ("infra", "the --locked shim"),
    "NROS_LINK_IP": ("infra", "link toggle"),
    "NROS_PLATFORMS_DIR": ("infra", "the ladder's own search path"),
    "NROS_PLATFORM_NAME": ("infra", "the ladder's own platform rung input"),
    "NROS_PX4_BRIDGE_GEN": ("infra", "codegen toggle"),
    "NROS_REPO_DIR": ("infra", "path"),
    "NROS_TRACE": ("infra", "debug flag"),
    "NROS_ZEPHYR_WORKSPACE": ("infra", "path"),
    # --- ladder: the zenoh tx tenant (phase-282), read in the build HELPER ---
    # --- sizing: the zenoh-pico entity caps and buffers. The largest single
    # --- family left, and the one the phase doc calls "the per-entity caps".
    "ZPICO_MAX_PUBLISHERS": ("derived", "entity cap; phase-392 is deciding whether it is derived from the declaration"),
    "ZPICO_MAX_SUBSCRIBERS": ("derived", "entity cap; phase-392"),
    "ZPICO_MAX_QUERYABLES": ("derived", "already a CHECKED OVERRIDE over a derived default (phase-392 W5.f)"),
    "ZPICO_MAX_SESSIONS": ("derived", "phase-392 poses it explicitly: joins the model, or stays a knob and that phase says so"),
    "ZPICO_MAX_LIVELINESS": ("derived", "entity cap; phase-392"),
    "ZPICO_MAX_PENDING_GETS": ("derived", "entity cap; phase-392"),
    "ZPICO_READ_TASK_PRIORITY": ("sizing", "transport-band priority (issue 0623)"),
    "ZPICO_LEASE_TASK_PRIORITY": ("sizing", "transport-band priority (issue 0623)"),
    # --- infra ---
    "NROS_DECLARED_INFRA_QUERYABLES": ("infra", "a COUNT the resolver passes down, not a knob"),
    "NROS_DECLARED_SERVICE_SERVERS": ("infra", "a COUNT the resolver passes down, not a knob"),
    "NROS_PICOLIBC_SYSROOT": ("infra", "path"),
    "NROS_RISCV64_PREFIX": ("infra", "toolchain prefix"),
    "NROS_SDK_STORE": ("infra", "path"),
    "NROS_SIZES_PROBE_TARGET_DIR": ("infra", "path"),
    "NROS_ZPICO_DEBUG": ("infra", "debug flag"),
    "ZPICO_NO_SMOLTCP": ("infra", "link toggle"),
    "ZPICO_PLATFORMS_TOML": ("infra", "the ladder's own platform file pointer"),
    # --- found once the scan followed the `build-helpers` feature ---
    "NROS_CYCLONEDDS_MAX_TYPES": (
        "derived",
        "already DERIVED from the SystemModel (msg=1/srv=2/action=8+3, next_pow2) "
        "and forwarded; the env is the override, not the source",
    ),
    "NROS_ENTRY_SPIN_MS": ("sizing", "entry spin period; a duration, same ladder shape"),
    # --- infra: pointers, flags and orchestration inputs ---
    "NROS_BOARD_TOML": ("infra", "the ladder's own board rung pointer"),
    # issue 1143 -- names WHICH `[[board]]` entry of a multi-entry file the
    # rung comes from (`nros-board-nuttx` declares two that differ in ISA).
    # A selector, never a size.
    "NROS_BOARD": ("infra", "which board entry the rung reads, beside NROS_BOARD_TOML"),
    # Surfaced when the reader matcher stopped enumerating helper names
    # (READ_CALLEES). These are read through `req`/`list`/`env_get`/`flag`,
    # which the old fixed list did not know, so the census had never counted
    # them and the W6 backlog was measured 29 names short.
    #
    # The board descriptor's facts: identity and paths the CLI hands the build,
    # which are what the ladder RESOLVES AGAINST rather than knobs it resolves.
    # phase-403 step 3 -- the entity inventory's per-kind counts. INFRA, not
    # sizing: nobody tunes these, they are what the image DECLARED, and the
    # arena derivation sums bytes over them. Same category as a board
    # descriptor fact -- something the ladder resolves AGAINST rather than a
    # knob it resolves.
    "NROS_ENTITY_COUNT_SUBSCRIPTION": ("infra", "declared entity count"),
    "NROS_ENTITY_COUNT_TIMER": ("infra", "declared entity count"),
    "NROS_ENTITY_COUNT_SERVICE_SERVER": ("infra", "declared entity count"),
    "NROS_ENTITY_COUNT_ACTION_CLIENT": ("infra", "declared entity count"),
    "NROS_ENTITY_COUNT_ACTION_SERVER": ("infra", "declared entity count"),
    # The receive payload class. DERIVED -- it is the largest bound among the
    # types this image subscribes to, which phase-403 W8 computes exactly. Read
    # here so the arena stops billing a subscription at the CLOSURE buffer,
    # which is also DEFAULT_TX_BUF and therefore larger.
    "NROS_SUBSCRIBER_BUFFER_SIZE": ("derived", "receive payload class"),
    "NROS_BOARD_ZEPHYR_ID": ("infra", "board descriptor fact"),
    "NROS_BOARD_TOOLCHAIN": ("infra", "board descriptor fact"),
    "NROS_BOARD_RUNNER": ("infra", "board descriptor fact"),
    "NROS_BOARD_RUST_TARGETS": ("infra", "board descriptor fact"),
    "NROS_BOARD_GATED_PKGS": ("infra", "board descriptor fact"),
    "NROS_BOARD_PRJ_CONF": ("infra", "board descriptor path"),
    "NROS_BOARD_BOARD_CONF": ("infra", "board descriptor path"),
    "NROS_BOARD_BOARD_OVERLAY": ("infra", "board descriptor path"),
    "NROS_BOARD_DEFAULT_RMW": ("infra", "board descriptor default, not a size"),
    "NROS_BOARD_DEFAULT_TRANSPORT": ("infra", "board descriptor default, not a size"),
    # Source and include directories.
    "NROS_C_INCLUDE": ("infra", "include dir"),
    "NROS_CPP_INCLUDE": ("infra", "include dir"),
    "NROS_PLATFORM_CFFI_INCLUDE": ("infra", "include dir"),
    "NROS_PLATFORM_POSIX_SRC": ("infra", "platform source dir"),
    "NROS_PLATFORM_FREERTOS_SRC": ("infra", "platform source dir"),
    "NROS_PLATFORM_THREADX_SRC": ("infra", "platform source dir"),
    "NROS_LAN9118_LWIP_DIR": ("infra", "driver source dir"),
    "NROS_VIRTIO_NET_NETX_DIR": ("infra", "driver source dir"),
    # Sizes, and none of them owned by another campaign: the only live doc
    # hits are usage records (phase-358 records `=8` as what recovered #271's
    # overflow; the dependency-weight audit names the RMW cap as a knob cmake
    # never resolves). Both are arguments FOR a board rung, the same shape the
    # ASI consumer's `NROS_MAX_PARAMETERS=256` had.
    # A RUNTIME trace toggle, not a build-time size — presence-tested with
    # `var_os` in `cffi/src/lib.rs`, never parsed. It became visible to this
    # census only when `nros-rmw-cffi` gained a `build-helpers` dependency and
    # its sources entered the scan; it configures nothing about the image.
    "NROS_RMW_TRACE_OPEN": ("infra", "runtime trace toggle, not a size"),
    "NROS_RMW_SUBSCRIBER_SLOTS": ("derived", "phase-412 W1 — COUNT_SUBSCRIPTION"),
    "NROS_EXTRA_BOARD_PATH": ("infra", "extra board search roots"),
    "NROS_HOME": ("infra", "path"),
    "NROS_MODEL_DIR": ("infra", "path"),
    "NROS_WORKSPACE": ("infra", "path"),
    "NROS_WORKSPACE_ROOT": ("infra", "path"),
    "NROS_LAUNCH_RESOLVE": ("infra", "resolver binary path"),
    "NROS_METADATA_PROBE_CACHE": ("infra", "cache path"),
    "NROS_RMW": ("infra", "the selected backend, not a size"),
    "NROS_OFFLINE": ("infra", "policy flag"),
    "NROS_NO_AUTO_SETUP": ("infra", "policy flag"),
    "NROS_SKIP_STALE_CHECK": ("infra", "policy flag"),
    "NROS_SUPPRESS_DEPRECATION": ("infra", "policy flag"),
    "NROS_DEBUG_BRINGUP_RESOLVER": ("infra", "debug flag"),
    "NROS_TRACE_ABI_GUARD": ("infra", "debug flag"),
}


def tracked(*globs):
    out = subprocess.check_output(
        ["git", "-C", ROOT, "ls-files", *globs], text=True
    ).split()
    return [os.path.join(ROOT, p) for p in out]


def read(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            return fh.read()
    except OSError:
        return ""


def _struct_fields(src, struct):
    """The `pub` field names of one struct. Factored out so the selftest can
    drive it on synthetic input rather than on the real file."""
    m = re.search(r"pub struct " + struct + r"\s*\{(.*?)\n\}", src, re.S)
    if not m:
        return []
    body = re.sub(r"(?m)^\s*(///|//).*$", "", m.group(1))
    return sorted(set(re.findall(r"(?m)^\s*pub ([a-z_][a-z0-9_]*)\s*:", body)))


# issue 0883's class, in a gate's own table — `KNOB_CLASS` was a per-PR conflict
# site. Three of eleven DIRTY pull requests on 2026-09-04 conflicted here and
# nowhere else, because it is one sorted dict that every knob change edits, and
# a phase-400 W6 migration flips entries from `sizing` to `ladder` by hand.
#
# A migration now edits ZERO lines of it: whether the ladder maps a knob is READ
# from the ladder's own source, which is the thing being changed anyway.
#
# Deliberately NOT the whole classification. "the ladder maps this env name" and
# "this knob is migrated" are different facts, and conflating them would hide
# remaining work: `NROS_SERVICE_TIMEOUT_MS` is mapped and NOT migrated — it has
# two readers, so `check-knob-single-reader` refuses it — and deriving its class
# from the mapping alone would silently move it out of the "still to migrate"
# count. Those cases are named in `LADDER_MAPPED_NOT_MIGRATED`, which is short,
# shrinks as they land, and is the only hand-kept part left.
LADDER_MAPPED_NOT_MIGRATED = {
    # Mapped by the ladder, still read in two places (zenoh build + the C
    # emitter), so it needs one emission point before it can claim a rung.
    "NROS_SERVICE_TIMEOUT_MS",
    # Mapped, but `nros-node`'s build script still reads the env directly.
    "NROS_SUBSCRIPTION_BUFFER_SIZE",
}


def ladder_env_keys():
    """Env names the ladder READS, from `platform_config.rs` itself.

    Three idioms, all in that one file: a match arm mapping a struct field to
    its env name, a direct `env("...")`, and `flag("...", default)` for a bool.
    The `#[cfg(test)]` module is cut first — its fixtures name knobs that the
    production reader does not, and counting those would classify a knob as
    laddered because a UNIT TEST mentioned it.
    """
    src = read(os.path.join(ROOT, "packages/tooling/nros-platform-config/src/platform_config.rs"))
    cut = src.find("#[cfg(test)]")
    if cut > 0:
        src = src[:cut]
    src = re.sub(r"(?m)^\s*///.*$", "", src)
    out = set()
    for pat in (
        r'=>\s*"((?:NROS|ZPICO)_[A-Z0-9_]+)"',
        r'\benv\(\s*"((?:NROS|ZPICO)_[A-Z0-9_]+)"',
        r'\bflag\(\s*"((?:NROS|ZPICO)_[A-Z0-9_]+)"',
    ):
        out |= set(re.findall(pat, src))
    return out


def ladder_knobs():
    """Fields on the typed `[knobs.*]` structs — the migrated knobs."""
    # phase-400 W6 — the ladder reader MOVED to a leaf crate so a driver build
    # script can read it without the `nros-board-common -> nros-platform ->
    # ... -> driver` cycle. `nros-board-common` re-exports it, so consumers are
    # unchanged; this path is not, and a stale one here reports "the ladder
    # holds 0 knobs" rather than "I cannot find the file".
    src = read(os.path.join(ROOT, "packages/tooling/nros-platform-config/src/platform_config.rs"))
    total = {}
    for struct in (
        "TxKnobs",
        "ExecutorKnobs",
        "TransportKnobs",
        "MemoryKnobs",
        "ParamKnobs",
        "RmwKnobs",
        "NetKnobs",
        "RuntimeKnobs",
        "WireKnobs",
        "XrceKnobs",
        "ZenohLimitKnobs",
    ):
        fields = _struct_fields(src, struct)
        if fields:
            total[struct] = fields
    return total


def kconfig_symbols():
    syms = set()
    for p in tracked("*Kconfig", "*Kconfig.*", "*/Kconfig"):
        for m in re.finditer(r"(?m)^\s*(?:menu)?config\s+(NROS_[A-Z0-9_]+)", read(p)):
            syms.add(m.group(1))
    return syms


# How a build script READS a knob, and how it does everything else with one.
#
# The census used to carry only the first list, and matching nothing else meant
# a build script that introduced a helper of its own silently dropped its knobs
# from the count: `nros-params/build.rs` wrapped its rungs in a local `knob()`
# and five names vanished while `--check` still passed, because the gate only
# fails on a name it SEES and cannot classify.
#
# Matching ANY call instead is the opposite error -- `.define("ZPICO_X", ..)`
# emits a C macro and `.with_env("NROS_X", ..)` sets a CHILD's environment;
# counting those as readers added 67 phantom names. So both lists are explicit
# and a callee in neither is a FAILURE: the same "a new knob forces a decision"
# rule this file already applies to knobs, applied to the idioms that read them.
READ_CALLEES = {
    # phase-403 step 3. Reads the environment like `env_usize`, but returns
    # Option: for an entity COUNT, absence and zero are different answers and
    # a default would erase the difference.
    "env_opt_usize",
    # phase-400 W6. `env_usize` with a ladder rung between the env and the
    # builtin.
    "env_usize_rung",
    "env", "env_get", "env_bool", "env_usize", "env_usize_min",
    "env_usize_compat", "env_or_repo_path", "env_path_or", "flag", "knob",
    "knob_usize", "knob_bool", "req", "list", "var", "var_os",
}

# Not reads. Named rather than ignored by default, so the failure below stays
# meaningful.
NON_READ_CALLEES = {
    "define",          # emits a C preprocessor macro
    "set_var", "remove_var", "with_env",  # WRITES an environment
    "push", "insert",  # builds a list / a fact map
    "get", "contains", "contains_key", "starts_with",  # inspects one
}

_CALL_RE = r'([A-Za-z_][A-Za-z0-9_:]*)\s*\(\s*&?"({prefix}_[A-Z0-9_]+)"'


def _calls_in(txt, prefix):
    """(callee, name) for every call taking a `<PREFIX>_*` literal first.

    Comments are stripped first: a knob named in a comment is documentation,
    not a reader, and counting it inflates the number this gate exists to
    track. Macros are excluded by the absence of `!` from the callee pattern,
    which keeps `panic!("NROS_X unset")` and friends out.
    """
    stripped = re.sub(r"(?m)^\s*(///|//).*$", "", txt)
    return [
        (m.group(1).rsplit("::", 1)[-1], m.group(2))
        for m in re.finditer(_CALL_RE.format(prefix=prefix), stripped)
    ]


def _env_names_in(txt, prefix):
    """`<PREFIX>_*` names READ from the environment in one source string.

    Factored out so the selftest drives it on synthetic input.
    """
    return {n for callee, n in _calls_in(txt, prefix) if callee in READ_CALLEES}


def unknown_idioms(sources, prefixes):
    """Callees that neither read nor demonstrably-don't. See READ_CALLEES."""
    out = {}
    for path in sources:
        txt = read(path)
        for prefix in prefixes:
            for callee, name in _calls_in(txt, prefix):
                if callee not in READ_CALLEES and callee not in NON_READ_CALLEES:
                    out.setdefault(callee, set()).add(name)
    return out


def build_env_sources():
    """Files that run at BUILD time and may read a knob.

    Not just `build.rs`: a build script that grew past a few lines moved its
    body into a helper crate, and the knobs moved with it. 21 `ZPICO_*` names
    live in `nros-zpico-build/src/`, and a census that scanned only `build.rs`
    reported six — so both this file's first number and the phase doc's
    original table undercounted the zenoh surface by a factor of four.

    Two conventions find them. The crate NAME (`*-build`, and everything under
    `packages/tooling/`), and — because a crate can be a build helper without
    saying so in its name — a declared `build-helpers` FEATURE. The second was
    added after a refactor moved the env-pointer reads out of a `build.rs` and
    into `nros-board-common`, whose name says "board" and whose
    `build-helpers` feature says what it actually is: the census lost sight of
    a knob because the code moved, which is the failure mode a name-based scan
    always has.
    """
    out = list(tracked("*build.rs"))
    out += [p for p in tracked("packages/*/*-build/src/*.rs", "packages/*/*/*-build/src/*.rs")]
    out += list(tracked("packages/tooling/*/src/*.rs"))
    for manifest in tracked("packages/*/*/Cargo.toml", "packages/*/Cargo.toml"):
        if "build-helpers" not in read(manifest):
            continue
        crate = os.path.dirname(manifest)
        rel = os.path.relpath(crate, ROOT)
        out += [str(q) for q in tracked(f"{rel}/src/*.rs")]
    return sorted(set(out))


def build_script_env(prefix):
    """`<PREFIX>_*` names a build-time source reads from the environment."""
    names = set()
    for p in build_env_sources():
        names |= _env_names_in(read(p), prefix)
    return names


def self_test() -> None:
    """Negative controls for both counters, on synthetic input.

    On the normal path, not behind a flag — `check-gate-selftests` requires it,
    on the reasoning that a control nobody runs decays into a comment. This
    census earns the scepticism twice over: it is a COUNTING gate, so a regex
    that silently matches too much or too little still prints a plausible
    number, and a wrong baseline is worse than none because it looks measured.
    """
    # The env matcher must see the idioms build scripts actually use...
    for src in (
        'let n = env_usize("NROS_EXECUTOR_MAX_CBS", 4);',
        'std::env::var("NROS_THING").ok()',
        'nros_zephyr_build::knob_usize("NROS_OTHER", &k, 1)',
    ):
        assert _env_names_in(src, "NROS"), f"selftest: missed a read in {src!r}"
    # ...and must not count a name it merely MENTIONS, or another prefix.
    for src in (
        '// NROS_EXECUTOR_MAX_CBS was read here once',
        'println!("set NROS_THING");',
        'let n = env_usize("ZPICO_TX_BATCH", 0);',
        # ...including the shapes that TOUCH a knob without reading one.
        'cfg.define("NROS_THING", "1");',
        'cmd.with_env("NROS_THING", "");',
        'flags.push("NROS_THING");',
    ):
        assert not _env_names_in(src, "NROS"), f"selftest: false positive on {src!r}"

    # An idiom in NEITHER list must be reported rather than silently dropped:
    # that silent drop is exactly what hid five knobs behind a local `knob()`.
    assert _calls_in('let n = brand_new_helper("NROS_THING", 4);', "NROS") == [
        ("brand_new_helper", "NROS_THING")
    ], "selftest: the call matcher missed an unknown idiom"
    assert not _env_names_in('let n = brand_new_helper("NROS_THING", 4);', "NROS"), (
        "selftest: an unknown idiom was counted as a read"
    )

    # The ladder counter reads STRUCT FIELDS, so it must not count a doc
    # comment, a non-`pub` field, or a field of a struct it was not asked for.
    src = (
        "pub struct TxKnobs {\n"
        "    /// pub not_a_field: usize,\n"
        "    pub batch: Option<bool>,\n"
        "    private: usize,\n"
        "}\n"
        "pub struct Unrelated {\n    pub nope: usize,\n}\n"
    )
    fields = _struct_fields(src, "TxKnobs")
    assert fields == ["batch"], f"selftest: ladder counter read {fields!r}"

    # The ladder-mapping reader: three idioms, and NOTHING from the test module.
    # Counting a test fixture would classify a knob as laddered because a unit
    # test mentioned it — the failure that makes a derived fact worse than a
    # hand-kept one.
    src = (
        '        "field" => "NROS_FROM_MATCH_ARM",\n'
        '        env("NROS_FROM_ENV_CALL")\n'
        '        flag("ZPICO_FROM_FLAG_CALL", BUILTIN)\n'
        '        /// env("NROS_IN_A_DOC_COMMENT")\n'
        '#[cfg(test)]\n'
        '        env("NROS_ONLY_IN_TESTS")\n'
    )
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        sub = os.path.join(d, "packages/tooling/nros-platform-config/src")
        os.makedirs(sub)
        with open(os.path.join(sub, "platform_config.rs"), "w") as fh:
            fh.write(src)
        global ROOT
        real, ROOT = ROOT, d
        try:
            got = ladder_env_keys()
        finally:
            ROOT = real
    assert got == {
        "NROS_FROM_MATCH_ARM",
        "NROS_FROM_ENV_CALL",
        "ZPICO_FROM_FLAG_CALL",
    }, f"selftest: ladder-mapping reader got {sorted(got)!r}"


def main() -> int:
    self_test()
    ladder = ladder_knobs()
    n_ladder = sum(len(v) for v in ladder.values())
    kconfig = kconfig_symbols()
    nros_env = build_script_env("NROS")
    zpico_env = build_script_env("ZPICO")

    seen = sorted(nros_env | zpico_env)
    mapped = ladder_env_keys()

    # The table must not restate what the ladder source already says. This is
    # the half that stops `KNOB_CLASS` being a conflict site: a migration edits
    # the ladder and nothing here.
    restated = sorted(
        n for n, c in KNOB_CLASS.items()
        if c[0] == "ladder" and n in mapped
    )
    if restated:
        print("config-knob-census: KNOB_CLASS restates the ladder for "
              f"{len(restated)} knob(s):")
        for n in restated:
            print(f"    {n}")
        print("  Whether the ladder maps a knob is read from")
        print("  packages/tooling/nros-platform-config/src/platform_config.rs.")
        print("  Delete these lines — a migration should edit the ladder, not this")
        print("  table, which is why three of eleven dirty PRs conflicted here.")
        return 1

    by_class = {}
    unclassified = []
    for n in seen:
        if n in mapped and n not in LADDER_MAPPED_NOT_MIGRATED:
            by_class.setdefault("ladder", []).append(n)
            continue
        cls = KNOB_CLASS.get(n)
        if cls is None:
            unclassified.append(n)
        else:
            by_class.setdefault(cls[0], []).append(n)

    print("phase-400 config census")
    print()
    print(f"  {'where configuration lives':<38} count")
    print(f"  {'-' * 38} -----")
    print(f"  {'knobs in the RFC-0049 ladder':<38} {n_ladder}")
    for struct, fields in sorted(ladder.items()):
        print(f"      {struct:<34} {len(fields)}")
    print(f"  {'Kconfig CONFIG_NROS_* declarations':<38} {len(kconfig)}")
    print()
    print(f"  build-script env names, by what they ARE ({len(seen)} total)")
    print(f"  {'-' * 38} -----")
    for cls, label in (
        ("sizing", "sizing knobs still to migrate  <-- W6"),
        ("derived", "another campaign is DERIVING these"),
        ("ladder", "already on the ladder"),
        ("infra", "paths / flags / ladder inputs (not knobs)"),
    ):
        print(f"  {label:<38} {len(by_class.get(cls, []))}")

    if "--verbose" in sys.argv:
        print()
        for cls in ("sizing", "derived", "ladder", "infra"):
            print(f"  {cls}:")
            for n in by_class.get(cls, []):
                print(f"    {n:<36} {KNOB_CLASS[n][1]}")

    unknown = unknown_idioms(build_env_sources(), ("NROS", "ZPICO"))
    if unknown:
        sys.stderr.write(
            "config-knob-census: FAILED — build script(s) touch a knob name "
            "through an idiom this census does not know:\n"
        )
        for callee, names in sorted(unknown.items()):
            sample = ", ".join(sorted(names)[:3])
            sys.stderr.write(f"    {callee}(...)  e.g. {sample}\n")
        sys.stderr.write(
            "  Add it to READ_CALLEES if it READS the environment, or to\n"
            "  NON_READ_CALLEES if it does something else with the name (emits\n"
            "  a C macro, sets a child's env, builds a list). Neither list may\n"
            "  be skipped: a helper in neither is counted as nothing, which is\n"
            "  how a local `knob()` wrapper took five migrated knobs out of\n"
            "  this census while the gate stayed green.\n"
        )
        return 1

    if unclassified:
        sys.stderr.write(
            "config-knob-census: FAILED — build scripts read env name(s) that "
            "KNOB_CLASS does not classify:\n"
        )
        for n in unclassified:
            sys.stderr.write(f"    {n}\n")
        sys.stderr.write(
            "  Classify each in scripts/check/config-knob-census.py. The point\n"
            "  is that a new knob forces a DECISION — is it a sizing knob for\n"
            "  the ladder, a size the message type should derive (phase-403),\n"
            "  or infrastructure? A heuristic would guess, and guessing is how\n"
            "  the backlog number stops meaning anything.\n"
        )
        return 1

    if "--check" in sys.argv and n_ladder < LADDER_FLOOR:
        sys.stderr.write(
            f"config-knob-census: FAILED — the ladder holds {n_ladder} knob(s), "
            f"below the {LADDER_FLOOR} recorded floor.\n"
            "  A knob leaving the ladder is a knob losing its platform and board\n"
            "  rung and its `nros config explain` row. If that is deliberate,\n"
            "  lower LADDER_FLOOR in this file and say why in the commit.\n"
        )
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
