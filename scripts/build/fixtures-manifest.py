#!/usr/bin/env python3
"""Read examples/fixtures.toml — the SSOT for fixture build options (Phase 177.9).

Consumed by both the fixture build recipes and the test-all staleness probe so
they build each fixture with identical features/target-dir/env.

  fixtures-manifest.py list --platform linux --lang rust [--rmw zenoh] [--id ID]
  fixtures-manifest.py list-workspaces --platform linux [--lang rust] [--id ID]
  fixtures-manifest.py validate-workspaces --platform linux

emits one record per matching entry, fields separated by the unit-separator
byte 0x1F (NOT tab — tab is IFS-whitespace, so bash `read` would collapse the
empty <env> field and shift the columns):

  <dir>\x1f<env>\x1f<cargo-args>

Read it in bash with `IFS=$'\x1f' read -r dir env args`. <env> is space-joined
`KEY=VAL` (or empty), <cargo-args> is the cargo build flags
(--no-default-features / --features a,b / --target-dir D / --target TRIPLE) —
the profile is added by the caller; word-split <cargo-args> into an argv array.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

SEP = "\x1f"

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # 3.10 and earlier
    import tomli as tomllib

DEFAULT_MANIFEST = "examples/fixtures.toml"

# This script's own repo root — `scripts/build/fixtures-manifest.py`. Used to
# locate the shell group resolver, which every other consumer reaches by a
# repo-relative path from a cwd it controls. Deriving it from `__file__` rather
# than from `--manifest` keeps `fixture-groups` correct when the manifest is
# passed from elsewhere.
SCRIPT_ROOT = Path(__file__).resolve().parents[2]

# The launch file a bringup resolves to when it declares no
# `[system].default_launch`. Mirrors the fallback in
# `nros_orchestration_ir::model_location::launch_to_model_rel`, which is the
# SSoT for the rule — see the note in `validate_workspace_fixture`.
DEFAULT_LAUNCH_FILE = "system.launch.xml"

# The RMW a `[[fixture]]` row that names none is built with — issue 0482.
#
# `rmw` is optional on `[[fixture]]` (67 of 240 buildable rows omit it: a rust
# leaf with default features gets the default backend, and for C/C++ rows the
# key doubles as `-DNROS_RMW=` so it is always written out). But a row's
# COORDINATE is `(platform, lang, rmw)`, and that is what every lane-scoped
# consumer filters on — the fixture BUILD (`--coords-from`) and the staleness
# GATE (`check-fixtures-stale.sh`, same filter).
#
# The default therefore has to be applied HERE, where rows are read, or a lane
# and a coverage check end up with two different answers for the same row. That
# is exactly what happened: `matrix_fixture_coverage.rs` applied
# `unwrap_or("zenoh")` and reported the rows modeled, while `matches_filters`
# compared a bare `None` against the lane's triples and matched nothing — so 67
# rows sat in NO coordinate-scoped lane, invisible to both the build and the
# gate that would have reported the build's omission. `just build-test-fixtures
# lane=tier2` then produced a stamp the preflight accepted and a run that
# mass-failed STALE.
#
# One spelling, in the reader every consumer goes through. `row_coord()` is the
# only place a row's coordinate is computed; `matrix_fixture_coverage.rs`
# consumes the `coords` subcommand rather than re-deriving it.
DEFAULT_RMW = "zenoh"

# The RMW vocabulary a coordinate may name. Mirrors `nros_tests::matrix::Rmw`;
# `matrix_fixture_coverage.rs::fixture_rows_all_modeled_by_matrix` is the gate
# that keeps the two in step (a row naming an RMW the matrix has no cell for
# fails there). Listed here so a TYPO fails as a typo, at validation, instead of
# silently landing the row on a coordinate no lane will ever select.
KNOWN_RMWS = ("zenoh", "cyclonedds", "xrce")


def load(path):
    with open(path, "rb") as f:
        return tomllib.load(f).get("fixture", [])


def load_workspace_fixtures(path):
    with open(path, "rb") as f:
        return tomllib.load(f).get("workspace_fixture", [])


# phase-319 W2 (issue 0351) — the compile-check lane's inventory.
#
# These are the fixtures `scripts/build/compile-check-fixtures.sh` builds. They
# used to live in six hardcoded arrays inside that script, each with its own
# colon-delimited positional format — which is why `check-fixtures-stale.sh`,
# which enumerates THIS manifest, could not see them (issue 0350 hid there for
# three days). AGENTS.md:79 already said they belong here.
#
# They are their own table rather than `[[fixture]]` rows because `list`'s record
# format is per-language and consumed positionally by `fixtures-build.sh`;
# overloading it would change that contract for 251 existing rows. The table is
# named for the LANE, not for a claim about the rows: ten are compile-intent
# checks with no runtime artifact, the other sixteen produce binaries and JSON
# that tests read or execute.
COMPILE_CHECK_BUILDERS = (
    "cargo-check",  # stage the tree, `cargo check`, stamp `.compile-ok`
    "cargo-build",  # stage the tree, `cargo build`, keep the binary
    "cmake-configure",  # cmake configure (+ build) into build/cmake-fixtures/<id>
    "cross-build",  # `cargo build --target <target>` for one or more profiles
    "cxx-syntax",  # `c++ -fsyntax-only` over a snippet; no artifact
    # phase-350 W2 (issue 0536) — the west lane's two shapes. Both build into
    # `build/west-fixtures/<id>` via `scripts/build/west-fixtures.sh`, NOT via
    # `compile-check-fixtures.sh`: west needs a provisioned Zephyr workspace, so
    # the lane that owns it runs them. They live in this table because that is
    # what they ARE — "configure (or build) succeeded and artifact X exists" —
    # and because a row here is what `output` was invented for.
    "west-build",  # full `west build`; `output` is the image
    "west-configure",  # `west build --cmake-only`; `output` is a configure artifact
)

# The two builders above, so the west lane and the compile-check lane can each
# ask "is this one of mine?" without restating the pair.
WEST_COMPILE_CHECK_BUILDERS = ("west-build", "west-configure")


def load_compile_check_fixtures(path):
    with open(path, "rb") as f:
        return tomllib.load(f).get("compile_check_fixture", [])


# phase-350 W5 (issue 0539) — a fixture id names BEHAVIOUR, not the work item
# that introduced it.
#
# CLAUDE.md already bans this for test names ("Test names describe behavior, not
# phase numbers [...] Phases go stale"), and the argument is stronger for a
# fixture id: it outlives the phase by longer and is typed by hand into
# `fixtures-build.sh --id`. Nine ids carried a work-item letter from a phase
# nobody has open (`n9_form1`, `o4_pkg_index`, `l9_register_c`, ...); they were
# renamed, and this stops the next one.
#
# The pattern is a LEADING letter+digits+underscore, which is what the
# work-item spellings share and what ordinary names do not — `mps2` and
# `qemu-arm-baremetal` contain digits without matching, because the digits are
# part of a real name rather than an index into a plan.
_WORK_ITEM_ID = re.compile(r"^[a-z]\d+_")
_PHASE_ISSUE_ID = re.compile(r"\b(phase|issue)[-_]?\d+", re.IGNORECASE)


def _reject_work_item_id(entry, fixture_id):
    if _WORK_ITEM_ID.match(fixture_id) or _PHASE_ISSUE_ID.search(fixture_id):
        _fail(
            entry,
            f"fixture id {fixture_id!r} names a work item, not a behaviour. "
            "Phases and issues go stale and the id outlives them (issue 0539); "
            "CLAUDE.md already forbids this for test names. Rename it after "
            "what the fixture DOES.",
        )


def validate_compile_check_fixture(entry):
    """Shape-check one compile-check row. Raises ValueError via `_fail`."""
    for key in ("id", "builder"):
        if not entry.get(key):
            _fail(entry, f"missing required key {key!r}")
    _reject_work_item_id(entry, entry["id"])

    builder = entry["builder"]
    if builder not in COMPILE_CHECK_BUILDERS:
        _fail(
            entry,
            f"unsupported builder {builder!r} "
            f"(expected one of: {', '.join(COMPILE_CHECK_BUILDERS)})",
        )

    # `cxx-syntax` probes a snippet resolved by id, so it carries no dir; every
    # other builder needs a source tree that exists.
    if builder == "cxx-syntax":
        if entry.get("dir"):
            _fail(
                entry, "cxx-syntax rows take no 'dir' (the snippet is resolved by id)"
            )
    else:
        if not entry.get("dir"):
            _fail(entry, f"missing required key 'dir' for builder {builder!r}")
        _require_dir(entry, Path(entry["dir"]), "fixture dir")

    if builder == "cross-build" and not entry.get("target"):
        _fail(entry, "missing required key 'target' for builder 'cross-build'")

    if builder in WEST_COMPILE_CHECK_BUILDERS:
        # `west_subdir` is the APP dir inside `dir` ('.' when the app is the
        # root); `west_board` may be empty, meaning "take it from board.cmake".
        if not entry.get("west_subdir"):
            _fail(entry, f"missing required key 'west_subdir' for builder {builder!r}")
        _require_dir(
            entry,
            Path(entry["dir"]) / entry["west_subdir"],
            "west app dir (dir/west_subdir)",
        )
        # `output` is the whole point of the split: it is what the stamp gates
        # on, and naming a configure artifact vs an image is what makes
        # "configure only" checkable rather than a claim in a comment (0536).
        if not entry.get("output"):
            _fail(entry, f"missing required key 'output' for builder {builder!r}")


# Where a `cxx-syntax` row's stamp is asserted. One directory, because these
# are all `nros-tests` integration tests.
CXX_CONSUMER_DIR = SCRIPT_ROOT / "packages/testing/nros-tests/tests"


def _cxx_rows_without_a_consumer(entries):
    """`cxx-syntax` ids no test names — issue 1032.

    The build stage deliberately does NOT fail on a snippet that will not
    compile: it leaves no `.compile-ok` and defers the report to the consuming
    test. That contract is only safe while a consumer EXISTS, and for
    `spin_until_future_complete` none ever did — it was compiled every run,
    failed every run since at least 2026-09-01, and could not be reported by
    anyone. A snippet with no consumer is build cost that cannot answer a
    question.

    A text search for the quoted id, not an import graph: the two consumers
    spell it differently (a literal argument in `cpp_api_drift.rs`, a `const`
    array in `platform_header_compile.rs`) and both are found this way.
    """
    if not CXX_CONSUMER_DIR.is_dir():
        return []
    # `git ls-files`, not a walk: these are tracked sources, and the repo's
    # `check-no-tracked-file-find` forbids the walk for the reason
    # `regenerate-bindings.sh` records — an index lookup instead of a
    # filesystem traversal.
    listed = subprocess.run(
        ["git", "ls-files", "-z", "--", str(CXX_CONSUMER_DIR)],
        cwd=SCRIPT_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    hay = "\n".join(
        (SCRIPT_ROOT / rel).read_text(encoding="utf-8", errors="replace")
        for rel in listed.split("\0")
        if rel.endswith(".rs")
    )
    return [
        e["id"]
        for e in entries
        if e.get("builder") == "cxx-syntax" and f'"{e["id"]}"' not in hay
    ]


def validate_compile_check_fixtures(entries):
    seen = {}
    for e in entries:
        validate_compile_check_fixture(e)
        fid = e["id"]
        if fid in seen:
            _fail(e, f"duplicate compile-check fixture id {fid!r}")
        seen[fid] = True
    orphans = _cxx_rows_without_a_consumer(entries)
    if orphans:
        raise ValueError(
            "cxx-syntax row(s) whose `.compile-ok` no test asserts: "
            + ", ".join(sorted(orphans))
            + f"\nThe build stage does not fail on a snippet that will not compile — "
            f"it defers the report to a consuming test (issue 1032). With no "
            f"consumer, nothing reports it at all.\nAdd an assertion under "
            f"{CXX_CONSUMER_DIR.relative_to(SCRIPT_ROOT)}/ that calls "
            f"`nros_tests::fixtures::require_compile_check(\"<id>\")`."
        )
    return len(seen)


def compile_check_record(entry):
    """One \x1f-separated record: id, builder, dir, pkg, manifest_dir, target,
    profiles, output, west_subdir, west_board, west_extra.

    Empty field = absent; the script defaults them. The three `west_*` tail
    fields are phase-350 W2's; every existing reader takes the first eight
    positionally and is unaffected by a longer record."""
    return SEP.join(
        (
            entry.get("id", ""),
            entry.get("builder", ""),
            entry.get("dir", ""),
            entry.get("pkg", ""),
            entry.get("manifest_dir", ""),
            entry.get("target", ""),
            ",".join(entry.get("profiles", [])),
            entry.get("output", ""),
            entry.get("west_subdir", ""),
            entry.get("west_board", ""),
            entry.get("west_extra", ""),
        )
    )


def cargo_args(entry, *, include_target_dir=True):
    args = []
    if entry.get("no_default_features"):
        args.append("--no-default-features")
    feats = entry.get("features")
    if feats:
        args += ["--features", ",".join(feats)]
    if include_target_dir and entry.get("target_dir"):
        args += ["--target-dir", entry["target_dir"]]
    if entry.get("target"):
        args += ["--target", entry["target"]]
    return " ".join(args)


def is_cargo_row(entry):
    """Does this row build with `cargo` (as opposed to `cmake`)?

    ONE spelling of the `lang in ("c", "cpp")` split, which decides three things
    that must agree: which record shape `list` emits, where `row_artifact_root`
    says the bytes land, and (phase-340 B2) whether the row can be redirected
    into a shared cargo `--target-dir` group at all. `fixtures-build.sh` only
    ever hands the cargo path to `nros_fixture_target_dir_flag`, so a cmake row
    must never appear in the group export — a cmake artifact redirected to
    `cargo-fixtures/<slug>` is a resolver looking somewhere nothing was written.
    """
    # phase-344 W2 — BUILDER-keyed, not language-keyed. `lang` was a proxy that
    # held only while every rust row built with cargo. It does not: the six
    # `qemu-riscv64-threadx/rust/*` cyclonedds rows are driven through
    # `build_threadx_cmake_rmw` into `build-cyclonedds/`, so a lang-keyed
    # predicate reported a `target/` dir nothing writes and left their real
    # output unattributable — the same shape P2 found in freertos, where a build
    # had no row at all.
    #
    # An explicit `builder = "cmake"` on the row is the SSoT (P2's precedent:
    # prefer a manifest fact over a rule at the call site). Absent it, the
    # language default stands, so every existing row is unaffected.
    if entry.get("builder") == "cmake":
        return False
    if entry.get("builder") == "cargo":
        return True
    # phase-350 W1 — a `west` row builds through `west build` into the ZEPHYR
    # WORKSPACE, so it is neither cargo nor cmake in the sense this predicate
    # decides. Answering "cargo" for the rust ones would hand them to
    # `nros_fixture_target_dir_flag` and claim their bytes land in
    # `<dir>/target`, which nothing writes -- the exact failure phase-344 W2
    # fixed for the threadx cyclonedds rows.
    if entry.get("builder") == "west":
        return False
    return entry.get("lang") not in ("c", "cpp")


def cmake_build_subdir(entry):
    """The cmake build dir a C/C++ `[[fixture]]` row builds into, relative to
    its `dir`.

    Deliberately keyed on the RAW `rmw` and not on `row_coord`: an rmw-less C/C++
    row builds into plain `build/`, and routing the zenoh default through here
    would move where the build writes. `row_coord`'s default answers "which lane
    is this row in"; this answers "where do its bytes land". Two questions.
    """
    return entry.get("build_subdir") or (
        f"build-{entry['rmw']}" if entry.get("rmw") else "build"
    )


def row_is_variant(entry):
    """Is this row a non-default CONFIGURATION of its example — phase-340 W2.d.

    THE single computation of "variant-ness", for the same reason `row_coord`
    and `row_artifact_root` are single computations of their facts (RFC-0070 R3:
    one derivation, consumed by every side).

    Expressed in R2's coordinate vocabulary — the row's own feature signature —
    NOT in an authored `target_dir`. R2 is explicit that "a new ad-hoc suffix is
    a bug, not a naming choice", and `target_dir = "target-zenoh"` is one of
    those spellings; reading it as a predicate made the manifest column load-
    bearing for a question it only ever answered by coincidence.

    The coincidence held because declaring an isolated target dir was, in
    2026-05, exactly what made a row expensive (issue 0029: the host-integration
    runner ran out of disk). phase-340 B3/wave 2 removed that: those rows now
    build into a shared group dir and `nros_fixture_strip_authored_target_dir`
    drops the authored flag before cargo sees it, so the column no longer
    describes cost, or anything else a consumer needs.

    Equivalence where it is consumed is GATED, not assumed — see
    `tests/core_only_predicate.sh`. Measured at the time of the swap: over the
    65 `linux`/`rust` rows both spellings select the same 12. The two diverge
    only on `qemu-arm-nuttx` rust rows, which carry a variant signature and no
    authored dir — a platform `--core-only` has never been invoked on.
    """
    if entry.get("target_dir"):
        return True
    # AUTHORED configuration fields, not the DERIVED `cargo_args()`. That
    # function synthesises flags from the coordinate (rmw, platform), so every
    # row with an `rmw` looks like a variant through it — measured: it selected
    # 38 of 65 linux/rust rows for the right total by luck, and would misclassify
    # rows on platforms where the coordinate carries more.
    #
    # `rmw` is deliberately NOT here: it is a COORDINATE, not a configuration.
    # A default-rmw row is core.
    return any(
        entry.get(k) for k in ("features", "no_default_features", "env", "cargo_args")
    )


def row_selector(entry):
    """The row's AUTHORED variant identity, as a 5-tuple of strings — issue 0517.

    `(dir, rmw, features, no_default_features, env)`. Measured over the 248
    manifest rows this is INJECTIVE, which `(dir, target_dir)` also is today —
    the point is that this one survives the column's deletion, because it names
    the configuration instead of a directory somebody had to invent for it.

    The test resolver needs it because the funnels it goes through
    (`build_example`, `build_example_rmw`) distinguish variants ONLY by the
    authored dir string, and that string is exactly what a shared group strips.
    A caller that knows WHICH fixture it wants can name this; it cannot name a
    `--target-dir` the build no longer passes.

    Emitted RAW rather than resolved: `rmw` here is what the row authored, NOT
    `row_coord`'s `DEFAULT_RMW`-filled value. The two answer different
    questions — the coordinate says which lane builds the row, this says which
    of a leaf's rows a caller means — and 64 rows author no `rmw` at all while
    every one of them has a coordinate. Conflating them would make those 64
    indistinguishable from their leaf's zenoh sibling.

    Four shapes exist and they separate cleanly, which is what makes a small
    constructor set on the Rust side possible:

        rmw + [rmw-<x>] + no_default_features   37 rows  RMW chosen by feature
        rmw + []                               144 rows  RMW baked by platform
        [] + []                                 64 rows  plain default row
        [] + features                            3 rows  tls / large-buf / zero-copy
    """
    rmw = str(entry.get("rmw") or "")
    features = ",".join(sorted(entry.get("features") or []))
    ndf = "1" if entry.get("no_default_features") else "0"
    env = ",".join(f"{k}={v}" for k, v in sorted((entry.get("env") or {}).items()))
    return (str(entry.get("dir") or "").rstrip("/"), rmw, features, ndf, env)


WEST_ROLES = (
    "talker",
    "listener",
    "service-server",
    "service-client",
    "action-server",
    "action-client",
)


def west_role(entry):
    """The leaf's role — authored, or the dir basename for the six role leaves.

    phase-350 W1: `examples/zephyr/rust/talker` IS the talker leaf, so the
    basename is the fact and `west_role` only exists for the leaves whose role
    is not a directory name (`logging-smoke`, and the entries when they land).
    """
    authored = entry.get("west_role")
    if authored:
        return authored
    name = (entry.get("dir") or "").rstrip("/").rsplit("/", 1)[-1]
    if name not in WEST_ROLES:
        _fail(
            entry,
            f"west row has no `west_role` and its dir basename {name!r} is not "
            f"one of the role leaves ({', '.join(WEST_ROLES)})",
        )
    return name


def west_lang_tag(entry):
    """The lang, verbatim — issue 0539 retired the `rs` short form (2026-08-13).

    This used to map `rust` -> `rs` for zephyr build-dir names, and it was not
    the only place that did: `nros_tests::zephyr::build_dir_for_example` carried
    the same two-line mapping for the TEST side. Two producers of one spelling,
    which is the drift 0539 is about — the two had to be retired together or the
    build and the resolver would name different directories.

    Kept as a function rather than inlined so the axis still has ONE name here
    if a future board needs a per-language dir token again.
    """
    return entry.get("lang")


def west_build_name(entry):
    """The west build-dir NAME, authored or derived as `build-<tag>-<role>-<rmw>`."""
    return entry.get("west_build_name") or (
        f"build-{west_lang_tag(entry)}-{west_role(entry)}-{row_coord(entry)[2]}"
    )


def west_id(entry):
    """The leaf id, authored or `zephyr/<board>/<lang>/<role>/<rmw>`."""
    return entry.get("west_id") or (
        f"zephyr/{entry.get('board', '')}/{entry.get('lang', '')}"
        f"/{west_role(entry)}/{row_coord(entry)[2]}"
    )


def row_builder(entry):
    """The row's effective builder: `cargo`, `cmake`, or `west` — phase-350 W1.

    ONE spelling of "what builds this row", exported by `coords` so a consumer
    never has to infer it. An explicit `builder` wins; otherwise the language
    default stands, which is the rule `is_cargo_row` has encoded since
    phase-344 W2.
    """
    declared = entry.get("builder")
    if declared in ("cargo", "cmake", "west"):
        return declared
    return "cargo" if is_cargo_row(entry) else "cmake"


def row_artifact_root(entry):
    """Where a row's built artifacts land, as a repo-relative path — phase-340 W3.

    THE single computation of a row's artifact root, for the same reason
    `row_coord` is the single computation of its coordinate. The fixture BUILD
    writes here (`cargo --target-dir` / `cmake -B <build_subdir>`) and the test
    RESOLVER reads from here, so it is also the only honest way to map a
    resolved binary path back to the manifest row that produced it — which is
    what a coordinate-scoped test RUN needs (issue 0482's "the resolver has no
    link back to the manifest row").

    Rust/mixed rows: `<dir>/<target_dir or "target">`.
    C/C++ rows:      `<dir>/<cmake_build_subdir>`.

    A row that authors no `target_dir` shares `<dir>/target` with every sibling
    row of the same dir, so this is a prefix a path may match ambiguously; the
    consumer resolves that by preferring the LONGEST match and by treating an
    ambiguous match as "not attributable" (fail closed — never skip). A tie
    among rows at the SAME coordinate is not ambiguous: the coordinate is what
    the caller asked for and every tied row gives it (issue 0922).

    An EMPTY root does NOT mean "fail closed" any more. That was true when west
    leaves had no lane route at all; issue 0713 gave them one
    (`require_west_leaf_in_lane`, keyed on the build-dir name both halves
    already agree on), so they are narrowed by coordinate like everything else
    and `row_is_lane_skippable` returns True for them.
    """
    d = (entry.get("dir") or "").rstrip("/")
    if not d:
        return ""
    # phase-350 W1 — a `west` row's artifacts do NOT land under `dir`. They land
    # in the Zephyr WORKSPACE (`$build_root/<west_build_name>`), whose root is a
    # host fact (`NROS_ZEPHYR_BUILD_ROOT`, the in-repo `zephyr-workspace/`, or a
    # sibling checkout) that no manifest can name. Return "" — UNATTRIBUTABLE —
    # rather than a repo-relative guess: `fixtures::lane` fails closed on an
    # empty root (never skips), and a wrong path would be worse than none, which
    # is the whole lesson of phase-344 W2. Giving these rows a real artifact
    # root is phase-350 W1.d; it needs the row to name the build-dir NAME and
    # the resolver to join it with the workspace root it already discovers.
    if entry.get("builder") == "west":
        return ""
    if not is_cargo_row(entry):
        return f"{d}/{cmake_build_subdir(entry)}"
    return f"{d}/{entry.get('target_dir') or 'target'}"


def env_str(entry):
    return " ".join(f"{k}={v}" for k, v in (entry.get("env") or {}).items())


def cmake_defs(entry):
    # `rmw` shorthand expands to -DNROS_RMW=<rmw>; explicit cmake_defs override.
    defs = {}
    if entry.get("rmw"):
        defs["NROS_RMW"] = entry["rmw"]
    defs.update(entry.get("cmake_defs") or {})
    return " ".join(f"-D{k}={v}" for k, v in defs.items())


def workspace_record(entry):
    # workspace record:
    # <id>\x1f<lang>\x1f<dir>\x1f<bringup>\x1f<entry>\x1f<build-subdir>
    # \x1f<target-dir>\x1f<codegen-out>\x1f<cmake -D defs>\x1f<env>\x1f<cargo-args>
    # \x1f<board>\x1f<conf-files>
    # \x1f<image>
    # board/conf_files are zephyr-only and `image` is set only on a migrated row
    # (phase-383 W9.b); rows without them emit empty strings so the field count
    # stays uniform (14 columns).
    return SEP.join(
        [
            entry["id"],
            entry["lang"],
            entry["dir"],
            entry["bringup"],
            entry_name(entry),
            entry.get("build_subdir", ""),
            entry.get("target_dir", ""),
            entry.get("codegen_out", ""),
            cmake_defs(entry),
            env_str(entry),
            cargo_args(entry, include_target_dir=False),
            entry.get("board", ""),
            ";".join(entry.get("conf_files", [])),
            entry.get("image", ""),
        ]
    )


_COORDS_CACHE = {}


# ── issue 0828 — a row the RUN cannot skip must be BUILT ────────────────────
#
# `--coords-from <lane>` narrows the build to a lane's cell cover, on the
# premise (CLAUDE.md, phase-340 W3) that "the RUN narrows to the same
# coordinates at fixture RESOLUTION time, so an out-of-lane fixture SKIPS
# rather than failing".
#
# That premise holds only for rows the resolver can ATTRIBUTE. `row_artifact_root`
# says so in its own docstring: a root shared by several rows is ambiguous, and
# the consumer treats an ambiguous match as "not attributable" — fail closed,
# NEVER skip. So an unattributable row is in every lane's run set regardless of
# its coordinate, while a coordinate-narrowed build omitted it.
#
# Measured: `examples/workspaces/c/build-workspace-fixtures` is shared by 14
# rows and that `build_subdir` name by 47 repo-wide. After a core-crate edit
# staled everything, `lane=tier2` + `just ci-matrix` passed the lane gate and
# then failed ~190 tests on stale fixtures — green only on a machine carrying
# older `lane=all` residue, which is precisely the property a lane exists to
# remove.
#
# So the filter below asks the resolver's question, not the cover's: skip a row
# only if the run could have skipped it too.
def _shared_artifact_roots(entries):
    """Artifact roots that are AMBIGUOUS — claimed by rows at differing coordinates.

    Issue 0922 — "more than one row" is not the test, and using it over-built 20
    rows. `attribute_path` treats a tie between rows at the SAME coordinate as
    unambiguous, because the coordinate is the only thing the caller asked for
    and every tied row gives the same answer. Two `zenoh` rows of one leaf are a
    tie the resolver resolves; a `zenoh` row and an `xrce` row are not.
    """
    seen, shared = {}, set()
    for e in entries:
        root = row_artifact_root(e)
        if not root:
            # West rows have no artifact root and are never path-attributed;
            # their lane narrowing is by coordinate alone, which is sound
            # because the resolver never tries to attribute them.
            continue
        coord = row_coord(e)
        if root in seen and seen[root] != coord:
            shared.add(root)
        seen.setdefault(root, coord)
    return shared


_SHARED_ROOTS_CACHE = {}


def row_is_lane_skippable(entry, all_entries, kind="fixture"):
    """Can a coordinate-scoped RUN skip this row? Only then may the build omit it.

    Issue 0922 — the answer depends on HOW the run attributes an artifact back
    to a row, and the two tables do it differently:

    * `[[fixture]]` rows are attributed BY PATH (`attribute_path`), so a root
      shared with a sibling is ambiguous, resolves to `None`, and fails closed.
    * `[[workspace_fixture]]` rows are attributed BY `id` (`attribute_workspace_id`),
      which is unique per row and gated as such. Their artifact roots are shared
      constantly and by design — 66 of 110 rows sit on 12 shared roots, because a
      workspace builds every one of its entries into one tree — and none of that
      sharing reaches attribution.

    Applying the path rule to workspace rows called 84 of them unskippable when
    the run skips 46, so `lane=tier2` rebuilt 38 workspace rows it never runs.
    That is the 0828 fix overshooting: 0828 was under-building, and a blanket
    root rule turns it into over-building on the other table.
    """
    if kind == "workspace_fixture":
        return True
    key = id(all_entries)
    if key not in _SHARED_ROOTS_CACHE:
        _SHARED_ROOTS_CACHE[key] = _shared_artifact_roots(all_entries)
    root = row_artifact_root(entry)
    return not root or root not in _SHARED_ROOTS_CACHE[key]


def _coords_for(path):
    """Parse a lane-coords file into a set of (platform, lang, rmw) triples."""
    if path not in _COORDS_CACHE:
        coords = set()
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = [x.strip() for x in line.split(",")]
                if len(parts) != 3:
                    raise SystemExit(
                        f"{path}: expected `platform,lang,rmw`, got {line!r}"
                    )
                coords.add(tuple(parts))
        if not coords:
            # An empty file would silently select NOTHING and the lane would look
            # instant rather than broken.
            raise SystemExit(f"{path}: no coordinates — refusing to select nothing")
        _COORDS_CACHE[path] = coords
    return _COORDS_CACHE[path]


GROUP_RESOLVER = "scripts/build/fixtures-target-dir.sh"


def shell_group_batch(records, *, root=None):
    """`[(slug, eligible), …]` for `[(platform, cargo_args, envstr), …]`.

    The group KEY is **not** computed here. `nros_fixture_group_slug` /
    `nros_fixture_group` in `scripts/build/fixtures-target-dir.sh` are the one
    derivation (RFC-0070 R3) and this shells into them, batched by that file's
    own `nros_fixture_group_batch` — because the alternative is a `cksum`
    reimplemented in Python, which is precisely the second spelling phase-340
    keeps deleting. (`check-fixture-groups.py` shells into the same batch driver
    for the same reason; the driver itself lives in the shell file so the two
    callers cannot drift.)

    One `bash` for the whole table: 240 spawns would make every consumer of this
    slower than the thing it is checking.
    """
    records = list(records)
    if not records:
        return []
    stdin = "".join(f"{p}{SEP}{a}{SEP}{e}\n" for p, a, e in records)
    res = subprocess.run(
        ["bash", "-c", f". {GROUP_RESOLVER}\nnros_fixture_group_batch\n"],
        cwd=str(root or SCRIPT_ROOT),
        input=stdin,
        capture_output=True,
        text=True,
    )
    if res.returncode != 0:
        raise SystemExit(
            f"fixtures-manifest.py: the shell group derivation failed:\n{res.stderr}"
        )
    out = res.stdout.splitlines()
    if len(out) != len(records):
        raise SystemExit(
            f"fixtures-manifest.py: the group derivation emitted {len(out)} slugs "
            f"for {len(records)} rows — the batch driver and this table disagree"
        )
    parsed = []
    for line in out:
        slug, _, elig = line.partition(SEP)
        parsed.append((slug, elig == "1"))
    return parsed


def row_coord(entry):
    """The `(platform, lang, rmw)` coordinate a row occupies — issue 0482.

    THE single computation of a fixture row's coordinate. Every consumer that
    asks "is this row in lane L?" must go through here, in either language:
    the lane build and the staleness gate via `matches_filters`/`coords`, the
    matrix cross-check via the `coords` subcommand. A second derivation with its
    own default is the defect this function exists to remove.

    `platform` and `lang` are required on every row (`validate_fixtures`);
    `rmw` falls back to `DEFAULT_RMW`.
    """
    return (
        entry.get("platform"),
        entry.get("lang"),
        entry.get("rmw") or DEFAULT_RMW,
    )


def matches_filters(entry, args, *, for_probe=False, all_entries=None, kind="fixture"):
    # `skip_build` rows stay in the manifest for documentation/inventory but
    # are intentionally NOT built as fixtures (e.g. an incomplete example).
    # Exclude them from both the build list and the stale probe — a row that
    # is never built can never be stale.
    if entry.get("skip_build"):
        return False
    if args.platform and entry.get("platform") != args.platform:
        return False
    # phase-350 W1 — `west` rows are OPT-IN. Every existing `list` consumer is
    # the cargo/cmake lane or its staleness probe (`fixtures-build.sh`,
    # `check-fixtures-stale.sh`, `drop-family-artifacts.sh`); handing them a row
    # that `west build`s into the Zephyr workspace would have them cargo-build a
    # Zephyr app, probe a `target/` nothing writes, or delete the wrong tree.
    # The zephyr lane asks for them by name (`--builder west`).
    if (entry.get("builder") == "west") != (getattr(args, "builder", None) == "west"):
        return False
    if getattr(args, "builder", None) in ("cargo", "cmake"):
        want_cargo = args.builder == "cargo"
        if is_cargo_row(entry) != want_cargo:
            return False
    if args.lang and entry.get("lang") != args.lang:
        return False
    # Issue 0482 — the `rmw` filter reads the row's COORDINATE, not its raw key,
    # for the same reason `--coords-from` does: `--rmw zenoh` and the coordinate
    # `(…, …, zenoh)` are the same question asked two ways, and answering them
    # differently is how a row ends up buildable by one caller and invisible to
    # another.
    if args.rmw and row_coord(entry)[2] != args.rmw:
        return False
    if args.id and entry.get("id") != args.id:
        return False
    coords_from = getattr(args, "coords_from", None)
    if coords_from:
        if row_coord(entry) not in _coords_for(coords_from):
            # Issue 0828 — out of the cover, but only OMIT it if the run could
            # have skipped it. An unattributable row (shared artifact root) is
            # run at every lane, so omitting it builds a lane whose own tests
            # then fail on staleness the lane never promised.
            if all_entries is None or row_is_lane_skippable(entry, all_entries, kind):
                return False
    # Issue #29 — `--core-only` excludes the isolated-`target_dir` variant cells
    # (the RMW/feature rebuilds that duplicate the dep graph + overrun disk).
    if getattr(args, "core_only", False) and row_is_variant(entry):
        return False
    if for_probe and entry.get("skip_probe"):
        return False
    return True


def _fail(entry, message):
    # `id` is required on workspace/compile-check rows but OPTIONAL on plain
    # `[[fixture]]` rows, most of which are identified only by their dir — so
    # fall back to it rather than reporting `<missing id>` for a row whose real
    # name is right there (issue 0482).
    fixture_id = entry.get("id") or entry.get("dir") or "<unidentifiable row>"
    raise ValueError(f"{fixture_id}: {message}")


def _require_file(entry, path, label):
    if not path.is_file():
        _fail(entry, f"missing {label}: {path}")


def _require_dir(entry, path, label):
    if not path.is_dir():
        _fail(entry, f"missing {label}: {path}")


def _load_toml(entry, path):
    try:
        with open(path, "rb") as f:
            return tomllib.load(f)
    except tomllib.TOMLDecodeError as exc:
        _fail(entry, f"{path}: invalid TOML: {exc}")


def _package_name(entry, path):
    package = _load_toml(entry, path).get("package") or {}
    return package.get("name")


def _workspace_members(entry, path):
    workspace = _load_toml(entry, path).get("workspace") or {}
    return workspace.get("members") or []


def _workspace_excludes(entry, path):
    """`[workspace] exclude` — packages under the root that cargo must not own.

    A Rust entry built by ANOTHER framework (west, idf.py) belongs here, not in
    `members`: it is compiled for a different target by a different driver, and
    listing it would make `cargo build` at the root try to build it. The
    generated root says so in its own comment. Before phase-383 W9 the eight
    hand-written roots also excluded their west entries, so this has always been
    the shape — the validator simply never looked at the second list, and the
    generated root is what made that visible (`workspace-rust-esp32` is a `lang
    = "rust"` row whose entry is driven by idf.py).
    """
    workspace = _load_toml(entry, path).get("workspace") or {}
    return workspace.get("exclude") or []


def entry_name(entry):
    """The entry PACKAGE name for a row — authored, or derived from its image.

    THE single derivation, matching `nros_cli_core::builder::entry::package_name`.
    The record, the artifact path and the test-side resolver all read it from
    here so a migrated row's binary name has one spelling.
    """
    authored = entry.get("entry")
    if authored:
        return authored
    image = entry.get("image") or ""
    return "{}_entry".format(re.sub(r"[-./]", "_", image))


def image_entry_package(entry):
    """The entry PACKAGE a row builds, honouring a west image's own `entry` key.

    RFC-0085 D4: on Zephyr the application is a HAND-WRITTEN package that west
    builds in place, and the image names it — `[image.zephyr] entry =
    "zephyr_entry"` — because six of the fourteen Zephyr images match more than
    one candidate and a first-match scan picks the wrong one. So a migrated west
    row derives its package from the IMAGE's declaration rather than restating
    it, which is what lets the row stop carrying `entry` at all.

    Falls back to [`entry_name`] — the generated `<image>_entry` spelling — for
    every other migrated row, whose entry package really is generated.
    """
    authored = entry.get("entry")
    if authored:
        return authored
    image = entry.get("image")
    if image:
        block = _image_block(entry, image)
        if block and block.get("entry"):
            return block["entry"]
    return entry_name(entry)


def _image_block(entry, image):
    """`[image.<id>]` from the row's bringup `system.toml`, or None."""
    root = Path(entry.get("dir") or "")
    bringup = entry.get("bringup")
    if not bringup:
        return None
    system_toml = root / bringup / "system.toml"
    if not system_toml.is_file():
        return None
    return (_load_toml(entry, system_toml).get("image") or {}).get(image)


def _require_image(entry, system_toml, image):
    """The `[image.<id>]` a migrated row builds must be declared.

    This is the migrated row's equivalent of `_require_dir(src/<entry>)`: it is
    the thing whose absence makes the row unbuildable, and it is checkable
    without running the builder.
    """
    table = _load_toml(entry, system_toml).get("image") or {}
    if image not in table:
        declared = ", ".join(sorted(table)) or "(none)"
        _fail(
            entry,
            f"names image {image!r}, which {system_toml} does not declare "
            f"(declared: {declared})",
        )


def _require_image_rmw(entry, system_toml, image):
    """The row's `rmw` must be one the image it names actually LINKS — issue 0831.

    `row_coord()` puts an RMW in every row's coordinate, `nros_lane` selects on
    it, and tier 2 reports coverage per coordinate. Nothing ever compared that
    claim to the image the row builds, and for two rows it was wrong for as long
    as they existed: `workspace-rust-native-cyclonedds` and
    `workspace-rust-native-xrce` named an image that declared no `rmw`, so the
    selection facade fell back to `[system] rmw` and both built ZENOH — measured
    at the time as 0 occurrences of Cyclone's `dds_` against 1916 of zenoh's.

    `rmw_coordinate_truth.rs` catches this on the ARTIFACT, which is the
    authority and stays. This is the same question asked of the DECLARATIONS, at
    `just check` speed and without a build: a row whose RMW no image can deliver
    is unbuildable-as-claimed the moment it is authored, and the lane that would
    have caught it may be a nightly away.

    The image's backend set is `image.rmw` (over `[image_defaults]`, falling back
    to `[system] rmw`) plus one per `[[domain]]` — deliberately the same union
    `facade::image_backends` compiles into the binary, which is why a BRIDGE
    passes: `bridge-cyclonedds` defaults to zenoh and declares a cyclonedds
    domain, so its one image links both and a row naming either is honest.

    One caveat, measured and currently inert: the CMAKE driver's fallback is the
    LITERAL `"zenoh"` (`CmakeRootSpec { rmw: image.rmw.unwrap_or("zenoh") }`),
    not `[system] rmw`, so the two drivers would disagree for an image that
    declares no `rmw` under a bringup whose system header names something else.
    Every shipped bringup declares `[system] rmw = "zenoh"`, so the two formulas
    give the same answer everywhere today; this reads the cargo one because that
    is the driver the issue is about.
    """
    system = _load_toml(entry, system_toml)
    table = system.get("image") or {}
    block = table.get(image) or {}
    defaults = system.get("image_defaults") or {}
    header = system.get("system") or {}

    selected = block.get("rmw") or defaults.get("rmw") or header.get("rmw")
    backends = {selected} if selected else set()
    backends.update(d.get("rmw") for d in system.get("domain", []) if d.get("rmw"))

    if not backends or entry["rmw"] in backends:
        return

    _fail(
        entry,
        f"declares rmw {entry['rmw']!r}, which the image {image!r} it builds "
        f"does not link (that image links: {', '.join(sorted(backends))}). "
        f"The RMW reaches a cargo build through the selection facade, which "
        f"reads `[image.{image}] rmw` over `[image_defaults]` over `[system] "
        f"rmw`, plus one backend per `[[domain]]` — so a row claiming an RMW "
        f"no image declares is a coordinate that tests something else "
        f"(issue 0831). Declare `rmw = {entry['rmw']!r}` on "
        f"`[image.{image}]` in {system_toml}, or correct the row.",
    )


def _system_default_launch(entry, path):
    system = _load_toml(entry, path).get("system") or {}
    return system.get("default_launch")


def _cmake_has_entry_target(text, entry_name):
    escaped = re.escape(entry_name)
    patterns = [
        # RFC-0048 / phase-287 W3 — the CURRENT spelling. Every C/C++/mixed entry
        # uses it; the detector knew only the older verbs, so all 47 of those rows
        # failed validation while building fine. Same drift as issue 0350: a verb
        # migration swept the CMakeLists and a checker that reads them did not
        # follow.
        rf"\bnano_ros_add_executable\s*\(\s*{escaped}\b",
        rf"\bnano_ros_entry\s*\([^)]*\bNAME\s+{escaped}\b",
        rf"\badd_executable\s*\(\s*{escaped}\b",
        rf"\badd_library\s*\(\s*{escaped}\b",
    ]
    return any(re.search(pattern, text, re.DOTALL) for pattern in patterns)


def _validate_rust_workspace(entry, root, entry_dir):
    workspace_manifest = root / "Cargo.toml"
    if not workspace_manifest.is_file():
        # A MIGRATED workspace has no tracked root: `nros build` generates it
        # from the discovered packages plus the `[image.*]` table (phase-383
        # W3.a), and it is gitignored build output. So it is absent in a fresh
        # clone and present only after a build — which makes "is this entry a
        # member?" unanswerable by a STATIC manifest check, and answering it
        # from whatever happens to be on disk is worse than not asking.
        #
        # This row keeps the `entry` form because its entry is hand-written and
        # driven by idf.py, not cargo; cargo membership is not what builds it.
        # The entry package's own existence is still checked by the caller.
        return
    _require_file(entry, workspace_manifest, "workspace Cargo.toml")

    member_names = set()
    member_basenames = set()
    for member in _workspace_members(entry, workspace_manifest):
        member_basenames.add(Path(member).name)
        member_manifest = root / member / "Cargo.toml"
        if member_manifest.is_file():
            name = _package_name(entry, member_manifest)
            if name:
                member_names.add(name)

    for excluded in _workspace_excludes(entry, workspace_manifest):
        member_basenames.add(Path(excluded).name)
        excluded_manifest = root / excluded / "Cargo.toml"
        if excluded_manifest.is_file():
            name = _package_name(entry, excluded_manifest)
            if name:
                member_names.add(name)

    expected = entry_name(entry)
    if expected not in member_names and expected not in member_basenames:
        _fail(
            entry,
            f"Rust entry {expected!r} is listed in workspace Cargo.toml "
            "neither as a member nor as an exclude",
        )

    _require_file(entry, entry_dir / "Cargo.toml", "entry Cargo.toml")


def _strip_cmake_comments(text):
    """Drop `#` comments so a gate matches CODE, not prose — phase-350 W1.

    CMake has no block comment worth modelling here, and `#` inside a quoted
    string is rare enough in these files that the simple rule is the honest one:
    everything from an unquoted `#` to end of line goes. Lines are kept (as
    blanks) so any future line-numbered error stays accurate.
    """
    out = []
    for line in text.split("\n"):
        in_str = False
        cut = len(line)
        for i, ch in enumerate(line):
            if ch == '"' and (i == 0 or line[i - 1] != "\\"):
                in_str = not in_str
            elif ch == "#" and not in_str:
                cut = i
                break
        out.append(line[:cut])
    return "\n".join(out)


def _validate_zephyr_workspace(entry, root, entry_dir):
    # A Zephyr west app is neither a cargo member nor a plain
    # add_executable/add_library target — it is driven by
    # find_package(Zephyr) + project() and links the entry via
    # rust_cargo_application() (Rust) or target_sources(app ...) (C/C++).
    entry_cmake = entry_dir / "CMakeLists.txt"
    _require_file(entry, entry_cmake, "entry CMakeLists.txt")

    # phase-350 W1 — match CODE, not prose. This read the raw text, so a token
    # in a COMMENT satisfied it: four of the six zephyr entries link with
    # `nano_ros_add_executable()` and merely MENTION `rust_cargo_application()`
    # in a comment, and passed on the mention. The one entry whose comments
    # happen not to name it (`mixed`) then failed — a gate that accepts four
    # rows for the wrong reason and rejects the fifth for a real one is worse
    # than no gate (the issue-0196 class).
    text = _strip_cmake_comments(entry_cmake.read_text(encoding="utf-8"))
    if "project(" not in text:
        _fail(entry, "entry CMakeLists.txt does not call project(...)")
    # The three shapes a Zephyr app is actually linked by here:
    #   rust_cargo_application()   zephyr-lang-rust, the pure-Rust entry
    #   target_sources(app ...)    plain Zephyr
    #   nano_ros_add_executable()  the RFC-0048 ament verb — what five of the
    #                              six workspace entries use
    linkers = ("rust_cargo_application", "nano_ros_add_executable")
    has_verb = any(re.search(rf"\b{v}\s*\(", text) for v in linkers)
    has_app_sources = bool(re.search(r"\btarget_sources\s*\(\s*app\b", text, re.DOTALL))
    if not (has_verb or has_app_sources):
        _fail(
            entry,
            "entry CMakeLists.txt does not link a Zephyr app (expected "
            "rust_cargo_application(), nano_ros_add_executable(), or "
            "target_sources(app ...))",
        )

    _require_file(entry, entry_dir / "prj.conf", "entry prj.conf")
    for name in entry.get("conf_files", []):
        _require_file(entry, entry_dir / name, f"conf file {name}")


def _validate_cmake_workspace(entry, root, entry_dir):
    root_cmake = root / "CMakeLists.txt"
    entry_cmake = entry_dir / "CMakeLists.txt"
    _require_file(entry, root_cmake, "workspace CMakeLists.txt")
    _require_file(entry, entry_cmake, "entry CMakeLists.txt")

    text = entry_cmake.read_text(encoding="utf-8")
    if not _cmake_has_entry_target(text, entry["entry"]):
        _fail(
            entry,
            "entry CMakeLists.txt does not define an obvious target "
            f"for {entry['entry']!r}",
        )


def validate_workspace_fixture(entry):
    required_keys = ("id", "platform", "lang", "dir", "rmw", "bringup")
    for key in required_keys:
        if not entry.get(key):
            _fail(entry, f"missing required key {key!r}")

    # phase-383 W9.b — `entry` and `image` are the two ways a row can name what
    # it builds, and exactly one of them is authored. `image` is the migrated
    # form: the entry package is generated by `nros build`, so its NAME is
    # derived (`<image>_entry`) rather than restated. Authoring both would put
    # one fact in two places, which is this repository's named defect class —
    # and the two could disagree, which is a row that builds one binary and
    # tests another.
    if bool(entry.get("entry")) == bool(entry.get("image")):
        _fail(
            entry,
            "a workspace row names EITHER `entry` (hand-written package under "
            "src/) OR `image` (an `[image.<id>]` the builder generates), never "
            "both and never neither",
        )

    lang = entry["lang"]
    if lang not in FIXTURE_LANGS:
        _fail(entry, f"unsupported workspace fixture lang {lang!r}")

    # Issue 0482 — the same vocabulary check `validate_fixture` makes. Both
    # tables carry coordinates and a typo has the same consequence in either:
    # the row lands on a coordinate no lane selects and no matrix cell holds, so
    # it is never built and never reported missing. Checking only one table is
    # the "gate narrower than the rule it enforces" shape (issue 0196).
    if entry["rmw"] not in KNOWN_RMWS:
        _fail(
            entry,
            f"unknown rmw {entry['rmw']!r} (expected one of: {', '.join(KNOWN_RMWS)})",
        )

    platform = entry["platform"]
    if platform == "zephyr" and not entry.get("board"):
        _fail(entry, "missing required key 'board' for zephyr workspace fixture")

    root = Path(entry["dir"])
    _require_dir(entry, root, "workspace dir")

    bringup_dir = root / entry["bringup"]
    _require_dir(entry, bringup_dir, "bringup dir")
    _require_file(entry, bringup_dir / "package.xml", "bringup package.xml")

    system_toml = bringup_dir / "system.toml"
    _require_file(entry, system_toml, "bringup system.toml")
    # A bringup declares its topology through LAUNCH, and only through launch.
    #
    # This used to accept a second way — a `config/system_model.yaml` on disk —
    # because phase-296 R4 had retired the launch bake. phase-330 reversed
    # that: the SystemModel is a BUILD ARTIFACT, `check-no-tracked-models`
    # rejects a committed one, and W7 deleted the last of them. That left the
    # model arm satisfiable only by an untracked build leftover, so this gate
    # passed or failed depending on whether someone had run `nros sync` in the
    # tree — and disagreed with `check-no-tracked-models` about the very file
    # it was accepting. A gate a stale artifact can satisfy is worse than no
    # gate.
    #
    # The default launch resolves the way the real consumers resolve it:
    # explicit `[system].default_launch`, else the conventional
    # `system.launch.xml`. That fallback is SSoT'd in
    # `nros_orchestration_ir::model_location::launch_to_model_rel` (shared by
    # `nros::main!(launch = …)`, `nros-build` and `nano_ros_entry(LAUNCH …)`) —
    # keep this line in step with it. It cannot delegate: that function is a
    # pure name mapping and never touches the filesystem, and existence is
    # exactly what this gate is for.
    default_launch = _system_default_launch(entry, system_toml) or DEFAULT_LAUNCH_FILE
    _require_file(
        entry,
        bringup_dir / "launch" / default_launch,
        f"default launch file (declare `[system].default_launch` in "
        f"{system_toml} to name a different one)",
    )

    # phase-383 W9.b — a row that names an `image` builds through `nros build`,
    # and its entry package is GENERATED. There is nothing under `src/` to
    # check; what must exist is the `[image.<id>]` declaration the builder
    # resolves. Validate THAT and stop, because every remaining check below
    # (`src/<entry>`, its `package.xml`, its membership in a hand-written
    # `[workspace]`) asks about files this row deliberately no longer has.
    if entry.get("image"):
        _require_image(entry, system_toml, entry["image"])
        _require_image_rmw(entry, system_toml, entry["image"])
        # ...unless it is a WEST row, where the application is hand-written and
        # west builds it in place (RFC-0085 D2/D4). Nothing is generated there,
        # so the package under `src/` is exactly as real as an unmigrated row's
        # — the difference is only that the IMAGE names it instead of the row.
        # Checking it here keeps the migrated west row under the same existence
        # contract it had before, which is the whole reason the `entry` key
        # could be dropped rather than merely stopped being read.
        if platform in ("zephyr", "zephyr-cortex-m"):
            pkg = image_entry_package(entry)
            if not pkg:
                _fail(
                    entry,
                    f"names image {entry['image']!r} but neither the row nor "
                    f"`[image.{entry['image']}] entry` in {system_toml} names "
                    f"the application package west must build (RFC-0085 D4)",
                )
            entry_dir = root / "src" / pkg
            _require_dir(entry, entry_dir, "entry dir")
            _require_file(entry, entry_dir / "package.xml", "entry package.xml")
            _validate_zephyr_workspace(entry, root, entry_dir)
        return

    entry_dir = root / "src" / entry["entry"]
    _require_dir(entry, entry_dir, "entry dir")
    _require_file(entry, entry_dir / "package.xml", "entry package.xml")

    if platform == "zephyr":
        _validate_zephyr_workspace(entry, root, entry_dir)
    elif lang == "rust":
        _validate_rust_workspace(entry, root, entry_dir)
    else:
        _validate_cmake_workspace(entry, root, entry_dir)


def validate_workspace_fixtures(entries):
    count = 0
    for entry in entries:
        validate_workspace_fixture(entry)
        count += 1
    return count


# The languages a `[[fixture]]` row may declare. Same set
# `validate_workspace_fixture` enforces; kept here rather than shared through a
# constant only because that function spells it inline and moving it is a
# separate change.
FIXTURE_LANGS = ("rust", "c", "cpp", "mixed")


def validate_fixture(entry):
    """Shape-check one `[[fixture]]` row — issue 0482.

    Only the COORDINATE keys, deliberately. `dir`/`features`/`target_dir` are
    already exercised by every build that reads them, and a wrong one fails
    loudly at build time. A missing `platform` or `lang` does not: the row keeps
    building under `lane=all` and quietly leaves every coordinate-scoped lane,
    which is issue 0482's whole shape. `rmw` stays optional — `row_coord`
    resolves it — but it must NAME a real RMW when present, or the row lands on
    a coordinate no lane and no matrix cell can ever hold.
    """
    for key in ("platform", "lang", "dir"):
        if not entry.get(key):
            _fail(entry, f"missing required key {key!r}")

    lang = entry["lang"]
    if lang not in FIXTURE_LANGS:
        _fail(
            entry,
            f"unsupported fixture lang {lang!r} "
            f"(expected one of: {', '.join(FIXTURE_LANGS)})",
        )

    rmw = entry.get("rmw")
    if rmw is not None and rmw not in KNOWN_RMWS:
        _fail(entry, f"unknown rmw {rmw!r} (expected one of: {', '.join(KNOWN_RMWS)})")

    # issue 0539 — same rule for a plain row's optional id.
    if entry.get("id"):
        _reject_work_item_id(entry, entry["id"])


def validate_fixtures(entries):
    count = 0
    for entry in entries:
        validate_fixture(entry)
        count += 1
    return count


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "command",
        choices=[
            "list",
            "list-workspaces",
            "validate-workspaces",
            # phase-319 W2 — the compile-check lane's inventory.
            "list-compile-checks",
            "validate-compile-checks",
            # Issue 0406 — classify one id across ALL row kinds, so a builder
            # that matched nothing can say WHY instead of exiting 0 silently.
            "describe-id",
            # Issue 0406 — the platform vocabulary, so a builder can reject a
            # typo instead of sweeping zero rows successfully.
            "list-platforms",
            # Issue 0482 — every row's COORDINATE, from `row_coord`. The one
            # export of the one computation: `matrix_fixture_coverage.rs`
            # consumes this instead of re-parsing fixtures.toml with a default
            # of its own, which is how the build and the coverage gate came to
            # disagree about 67 rows.
            "coords",
            # phase-340 B2 — where each cargo row's artifacts ACTUALLY land:
            # its leaf artifact root paired with the shared `--target-dir`
            # group the build redirects it into. The Rust fixture resolver
            # consumes this instead of re-deriving a group key it cannot spell
            # (the key is a `cksum` over the row's variant signature).
            "fixture-groups",
            # phase-350 W1 — the zephyr west lane's leaf table, so
            # `zephyr-fixture-leaves.sh` iterates the MANIFEST instead of the
            # nested bash loops in `fixture-matrix.sh` that were a second
            # spelling of it (issue 0535).
            "west-leaves",
            # Issue 0482 — shape-check `[[fixture]]` rows. `validate-workspaces`
            # has required `platform`/`lang`/`rmw` on workspace rows since
            # phase-295; plain rows had no validator at all, so a row missing
            # `platform` or `lang` had NO coordinate and fell out of every lane
            # silently.
            "validate-fixtures",
        ],
    )
    p.add_argument("--manifest", default=DEFAULT_MANIFEST)
    p.add_argument("--platform")
    p.add_argument("--lang")
    p.add_argument("--rmw")
    p.add_argument("--id")
    # The test-all staleness probe builds with the default (stable) toolchain
    # and can't replicate a recipe-injected platform toolchain (e.g. the ESP32
    # nightly + build-std). Such cells set `skip_probe = true` so --for-probe
    # omits them — otherwise the probe rebuilds them under the wrong toolchain
    # every preflight (toolchain-fingerprint thrash → permanent false-stale).
    p.add_argument("--for-probe", action="store_true")
    # Phase 226.D — prepend `<platform>\x1f` to each rust cargo record so
    # the stale probe (scripts/test/rust-fixture-stale.sh) can feed the
    # shared fixture-target-dir resolver, which keys on platform. The
    # build path (fixtures-build.sh) already knows the platform from its
    # CLI arg, so it does NOT pass this flag and keeps the 3-field record.
    p.add_argument("--with-platform", action="store_true")
    # Issue #29 — `--core-only` restricts to the default-config fixtures: rows
    # that do NOT declare an isolated `target_dir`. The RMW/feature variant
    # cells (TLS, safety-e2e, zero-copy, zenoh, xrce, large-buf) each author an
    # isolated `target_dir`, so each is a full standalone rebuild of the dep
    # graph — the duplication that overruns the host-integration runner disk.
    # Those variants are exercised by other lanes (platform-ci native cells,
    # the RMW-specific lanes); the host-integration lane only needs the
    # default-RMW per-example fixtures + the workspace fixtures, so it builds
    # with `--core-only` and the variant-spawning tests `skip!` here.
    # phase-318 W4.d — restrict to a CI lane's fixture coordinates. The file
    # holds `platform,lang,rmw` lines (see `lane-coords`), so a lane's build, its
    # staleness gate and its test selection all derive from ONE computation
    # instead of three hand-kept lists.
    p.add_argument(
        "--coords-from",
        metavar="FILE",
        help="only rows whose (platform,lang,rmw) appears in FILE (one triple per line)",
    )
    p.add_argument("--core-only", action="store_true")
    # phase-319 W2 — narrow `list-compile-checks` to one builder, so the shell
    # lane can keep its per-builder loops.
    #
    # phase-344 W2 — `list` reuses this SAME flag for cargo|cmake rather than
    # declaring a second one (argparse rejects the duplicate, which is how the
    # collision was found). One flag, two consumers, values disjoint.
    p.add_argument("--builder")
    a = p.parse_args()

    if a.command == "list-platforms":
        # Issue 0406. Every platform naming at least one buildable row, from
        # both the single-node and workspace tables. Compile-check rows carry
        # no platform.
        platforms = set()
        for e in load(a.manifest):
            if e.get("platform"):
                platforms.add(e["platform"])
        for e in load_workspace_fixtures(a.manifest):
            if e.get("platform"):
                platforms.add(e["platform"])
        for name in sorted(platforms):
            sys.stdout.write(f"{name}\n")
        return

    if a.command == "west-leaves":
        # phase-350 W1 — one record per `builder = "west"` row, in manifest
        # order, carrying what the emitter needs to build the leaf:
        #
        #   <board>\x1f<lang>\x1f<lang_tag>\x1f<rmw>\x1f<role>\x1f<dir>
        #   \x1f<build_name>\x1f<id>\x1f<zenoh_locator>\x1f<xrce_port>
        #   \x1f<cyclone_domain>\x1f<conf_files>\x1f<reserved>\x1f<ws_dir>
        #   \x1f<nros_image>\x1f<coord>
        #
        # IDENTITY only. The isolation triple (locator / xrce port / cyclone
        # domain) is emitted EMPTY for the six role leaves, because those derive
        # from the allocator offsets and that formula keeps its one home in
        # `zephyr-fixture-leaves.sh` beside `nros_tests::alloc`'s mirror --
        # exporting a computed port here would make the manifest a second
        # spelling of the allocator, which is the defect this phase removes.
        # The other leaves already held literals in the script, so their rows
        # carry the same literal and the emitter uses it verbatim.
        #
        # `conf_files` is the RELATIVE overlay list only; the absolute NSOS /
        # board tail is a host path the script appends.
        # BOTH coordinate-bearing tables. The 12 workspace ENTRY leaves are
        # Workspace cells (`fixture_rows_all_modeled_by_matrix` rejects them as
        # plain rows), so they live in `[[workspace_fixture]]` — but the west
        # lane builds them exactly like every other leaf, from the entry app dir
        # `<dir>/src/<entry>`, the same path `validate-workspaces` checks. A
        # zephyr workspace row IS a west row; that table has no `builder` key to
        # say so, so the platform is the discriminator.
        west_rows = [e for e in load(a.manifest) if e.get("builder") == "west"]
        for e in load_workspace_fixtures(a.manifest):
            if e.get("platform") not in ("zephyr", "zephyr-cortex-m"):
                continue
            e = dict(e)
            # phase-383 W9.b — the WORKSPACE root, kept before `dir` is
            # rewritten to the application. `nros build` is addressed from the
            # workspace, not from the app, so a migrated row needs both and the
            # rewrite destroys one of them.
            e["ws_dir"] = e["dir"].rstrip("/")
            e["dir"] = f"{e['dir'].rstrip('/')}/src/{image_entry_package(e)}"
            e.setdefault("west_role", "entry")
            west_rows.append(e)

        for e in west_rows:
            if a.platform and e.get("platform") != a.platform:
                continue
            # phase-350 W1.b — lane narrowing, through `row_coord` like every
            # other lane-scoped build. This is the whole point of the rows: the
            # zephyr lane could previously only be taken or left as a MODULE, so
            # `lane=tier2` -- which needs two zephyr coordinates -- built all 70
            # leaves and serial-added ~40 min to the sweep (issue 0509).
            if a.coords_from and row_coord(e) not in _coords_for(a.coords_from):
                continue
            # The isolation triple is emitted when the row AUTHORED one, not
            # when the role happens to be a role leaf: the mps2 witness leaves
            # ARE `talker`s, yet their locator (`tcp/10.0.2.2:106xx`) is a
            # literal the native_sim formula cannot produce -- it is a different
            # board's allocator slot, on the SLIRP host address. Keying this on
            # the role dropped it and would have silently rebuilt those three
            # leaves against the wrong router.
            authored = (
                e.get("west_zenoh_locator"),
                e.get("west_xrce_agent_port"),
                e.get("west_cyclone_domain"),
            )
            # The leaf's rmw LABEL, which is not always its coordinate: the
            # logging-smoke leaf omits `rmw` (so it coordinates at the `zenoh`
            # default, like every other rmw-less row) but the emitter has always
            # spelled its record `default`, and this rewire must not change a
            # single emitted byte.
            rmw_label = e.get("rmw") or "default"
            sys.stdout.write(
                SEP.join(
                    (
                        str(e.get("board", "")),
                        str(e.get("lang", "")),
                        west_lang_tag(e),
                        rmw_label,
                        west_role(e),
                        str(e.get("dir", "")),
                        west_build_name(e),
                        west_id(e),
                        *(str(v or "") for v in authored),
                        ";".join(e.get("conf_files", [])),
                        # (issue 0549 — a `west_bare` field rode here, blanking
                        # a leaf's cmake defs and staleness signature. Exactly
                        # one row used it, and that row turned out to be a
                        # vestigial duplicate; the field went with the fix. The
                        # trailing position is kept EMPTY rather than removed so
                        # the record width does not change under readers.)
                        "",
                        # phase-383 W9.b — the retarget columns. EMPTY on every
                        # unmigrated row, which is how the emitter tells "build
                        # this leaf with `west build`" from "build it with
                        # `nros build <bringup>:<image>`". `ws_dir` is the
                        # workspace `nros build` is addressed from; the image is
                        # QUALIFIED here for the same reason
                        # `workspace-fixtures-build.sh` qualifies it — one image
                        # id can be declared by two bringups of one workspace.
                        str(e.get("ws_dir", "")),
                        (
                            f"{Path(e['bringup']).name}:{e['image']}"
                            if e.get("image") and e.get("bringup")
                            else ""
                        ),
                        # issue 1016 — the leaf's COORDINATE, from `row_coord`,
                        # the same function `--coords-from` filters this record
                        # on three lines above.
                        #
                        # It is emitted rather than left to the reader because
                        # the reader was deriving it: `fixtures::lane::
                        # west_leaves` rebuilt the triple from `board` and the
                        # rmw LABEL with its own two special cases
                        # (`mps2_an385` -> `zephyr-cortex-m`, `default` ->
                        # `zenoh`). Two derivations of one coordinate is the
                        # defect `row_coord` exists to remove, and here the two
                        # sides are the BUILD's skip predicate and the RUN's --
                        # so a disagreement is a leaf the lane omits and the run
                        # demands, which is issue 0828's shape for this table.
                        # They agreed when measured; nothing made them.
                        ",".join(row_coord(e)),
                    )
                )
                + "\n"
            )
        return

    if a.command == "coords":
        # Issue 0482. One
        # `<kind>\x1f<platform>\x1f<lang>\x1f<rmw>\x1f<dir>\x1f<id>\x1f<artifact_root>\x1f<builder>`
        # line per BUILDABLE row of both coordinate-bearing tables, with the
        # coordinate resolved by `row_coord` — the same function the lane filter
        # uses. `skip_build` rows are omitted for the same reason
        # `matches_filters` omits them: a row nothing builds occupies no
        # coordinate.
        #
        # `dir` rides along so a consumer can NAME the offending row instead of
        # printing a bare triple; the coverage gate's failure messages were
        # unreadable without it.
        #
        # phase-340 W3 — `id` and `artifact_root` ride along so a consumer can
        # go the OTHER way: from a resolved fixture artifact back to the row
        # that produced it, and hence to its coordinate. That is what lets a
        # coordinate-scoped test RUN skip exactly the rows its lane's BUILD
        # omitted, deciding both from this one table with this one `row_coord`.
        #
        # phase-350 W1 — `builder` rides along for the same reason: not every row
        # is narrowed by the same lane. The zephyr west rows are selected
        # module-level today, not by coordinate, so the run-side predicate must
        # be able to tell them apart from a cargo/cmake row it CAN skip. Without
        # this field the consumer would have to re-derive the builder from
        # `lang`, which is the proxy phase-344 W2 already found wrong.
        fixture_rows = load(a.manifest)
        workspace_rows = load_workspace_fixtures(a.manifest)
        for kind, rows in (
            ("fixture", fixture_rows),
            ("workspace_fixture", workspace_rows),
        ):
            for e in rows:
                if e.get("skip_build"):
                    continue
                platform, lang, rmw = row_coord(e)
                sys.stdout.write(
                    SEP.join(
                        (
                            kind,
                            str(platform or ""),
                            str(lang or ""),
                            str(rmw or ""),
                            str(e.get("dir", e.get("id", ""))),
                            str(e.get("id", "")),
                            row_artifact_root(e),
                            row_builder(e),
                        )
                    )
                    + "\n"
                )
        return

    if a.command == "fixture-groups":
        # phase-340 B2. One
        # `<artifact_root>\x1f<platform>\x1f<group_slug>\x1f<eligible 0|1>`
        # line per buildable CARGO `[[fixture]]` row: where the row's artifacts
        # would land if nothing were shared (`row_artifact_root`, the same
        # function `coords` exports and the test resolver already inverts), and
        # the group the fixture BUILD actually redirects it into.
        #
        # This is the join the Rust fixture resolver cannot make for itself.
        # `build_example("native/rust/talker", …)` and
        # `build_example_rmw(…, Rmw::Zenoh)` distinguish variants ONLY by the
        # authored dir string (`target/` vs `target-zenoh/`) — which is exactly
        # the string a group strips — so the variant has to come from the
        # manifest ROW. And the slug itself is a `cksum` owned by
        # `fixtures-target-dir.sh`, so Rust must not re-derive it either.
        # Manifest row + shell key, joined once, here.
        #
        # Deliberately NOT filtered by --platform/--lang, like `coords`: a
        # filtered table would make some rows silently un-redirectable, and the
        # resolver would look in the leaf dir the build no longer writes to.
        #
        # cmake rows are excluded (`is_cargo_row`) because `fixtures-build.sh`
        # only routes the cargo path through `nros_fixture_target_dir_flag`.
        rows = [
            e for e in load(a.manifest) if not e.get("skip_build") and is_cargo_row(e)
        ]
        derived = shell_group_batch(
            (e.get("platform", ""), cargo_args(e), env_str(e)) for e in rows
        )
        for e, (slug, eligible) in zip(rows, derived):
            sys.stdout.write(
                SEP.join(
                    (
                        row_artifact_root(e),
                        str(e.get("platform", "")),
                        slug,
                        "1" if eligible else "0",
                    )
                    + row_selector(e)
                    # issue 0517 step 1 — the row's COORDINATE, so a resolver
                    # that already selected the row can ask the lane about it
                    # directly instead of handing back a path for
                    # `attribute_path` to re-derive the row from. `row_coord` is
                    # still the single computation; this only carries it.
                    + row_coord(e)
                )
                + "\n"
            )
        return

    if a.command == "validate-fixtures":
        try:
            count = validate_fixtures(load(a.manifest))
        except ValueError as exc:
            sys.stderr.write(f"fixtures-manifest.py: {exc}\n")
            sys.exit(1)
        sys.stdout.write(f"validated {count} fixture row(s)\n")
        return

    if a.command == "describe-id":
        # Issue 0406. Prints one `kind<SEP>platform<SEP>lang<SEP>rmw` line per
        # row carrying this id, across every kind, IGNORING --platform/--lang
        # (the caller already knows those did not match; what it needs is where
        # the id actually lives). No output = the id exists nowhere.
        #
        # Issue 0482 — the triple is the row's resolved COORDINATE (`row_coord`),
        # not its raw keys. This exists to explain why a filter did not match,
        # and a diagnostic that reports a different coordinate from the filter it
        # is explaining is this very bug in miniature: an rmw-less row would say
        # `rmw=` while the filter sees `zenoh`.
        if not a.id:
            sys.stderr.write("fixtures-manifest.py: describe-id needs --id\n")
            sys.exit(2)
        for kind, rows in (
            ("fixture", load(a.manifest)),
            ("workspace_fixture", load_workspace_fixtures(a.manifest)),
        ):
            for e in rows:
                if e.get("id") != a.id:
                    continue
                platform, lang, rmw = row_coord(e)
                sys.stdout.write(
                    SEP.join(
                        (
                            kind,
                            str(platform or ""),
                            str(lang or ""),
                            str(rmw or ""),
                        )
                    )
                    + "\n"
                )
        for e in load_compile_check_fixtures(a.manifest):
            if e.get("id") == a.id:
                sys.stdout.write(
                    SEP.join(
                        ("compile_check_fixture", "", "", str(e.get("builder", "")))
                    )
                    + "\n"
                )
        return

    if a.command in ("list-compile-checks", "validate-compile-checks"):
        entries = []
        for e in load_compile_check_fixtures(a.manifest):
            if a.id and e.get("id") != a.id:
                continue
            if a.builder and e.get("builder") != a.builder:
                continue
            entries.append(e)

        if a.command == "validate-compile-checks":
            try:
                # Validate the WHOLE table, not the filtered view — a filter must
                # never hide a malformed row.
                count = validate_compile_check_fixtures(
                    load_compile_check_fixtures(a.manifest)
                )
            except ValueError as exc:
                sys.stderr.write(f"fixtures-manifest.py: {exc}\n")
                sys.exit(1)
            sys.stdout.write(f"validated {count} compile-check fixture(s)\n")
            return

        for e in entries:
            sys.stdout.write(f"{compile_check_record(e)}\n")
        return

    if a.command in ("list-workspaces", "validate-workspaces"):
        entries = []
        ws_rows = load_workspace_fixtures(a.manifest)
        for e in ws_rows:
            if not matches_filters(e, a, all_entries=ws_rows, kind="workspace_fixture"):
                continue
            if a.for_probe and e.get("skip_probe"):
                continue
            entries.append(e)

        if a.command == "validate-workspaces":
            try:
                count = validate_workspace_fixtures(entries)
            except ValueError as exc:
                sys.stderr.write(f"fixtures-manifest.py: {exc}\n")
                sys.exit(1)
            sys.stdout.write(f"validated {count} workspace fixture(s)\n")
            return

        for e in entries:
            sys.stdout.write(f"{workspace_record(e)}\n")
        return

    fixture_rows = load(a.manifest)
    for e in fixture_rows:
        if not matches_filters(e, a, for_probe=a.for_probe, all_entries=fixture_rows):
            continue
        if not is_cargo_row(e):
            # cmake record: <dir>\x1f<build-subdir>\x1f<cmake -D defs>\x1f<target>
            # `cmake_build_subdir` is shared with `row_artifact_root`, so where
            # the build WRITES and where the resolver's attribution LOOKS are
            # one expression (phase-340 W3).
            sub = cmake_build_subdir(e)
            sys.stdout.write(
                f"{e['dir']}{SEP}{sub}{SEP}{cmake_defs(e)}{SEP}{e.get('target', '')}\n"
            )
        else:
            # cargo record: <dir>\x1f<env>\x1f<cargo-args>
            # With --with-platform: <platform>\x1f<dir>\x1f<env>\x1f<cargo-args>
            prefix = f"{e.get('platform', '')}{SEP}" if a.with_platform else ""
            sys.stdout.write(
                f"{prefix}{e['dir']}{SEP}{env_str(e)}{SEP}{cargo_args(e)}\n"
            )


if __name__ == "__main__":
    main()
