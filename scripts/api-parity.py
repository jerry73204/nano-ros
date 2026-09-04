#!/usr/bin/env python3
"""Phase 379 — how the nano-ros user API differs from the ROS 2 client library it mirrors.

nano-ros claims to be rclc / rclcpp / rclrs in shape, so a ROS 2 developer can
read and write it, and so a ported source file compiles with a build-glue change
rather than a rewrite. RFC-0036 catalogs the divergences that claim permits.

That catalog is PROSE, and prose about an API goes stale silently -- RFC-0036
itself shipped calling the Rust error `RclrsError` when the type had been
`NanoRosError` for months. This tool is the same catalog with a build behind it:
it extracts both surfaces from their actual sources and reports every item that
does not correspond.

    scripts/api-parity.py                 # report all three languages
    scripts/api-parity.py --lang cpp      # one language
    scripts/api-parity.py --show same     # include the matching rows too
    scripts/api-parity.py --suggest-renames   # pair up look-alike unmatched names
    scripts/api-parity.py --include-internal  # do not filter the ROS 2 side to public API
    scripts/api-parity.py --topic pubsub   # one stage, all three languages
    scripts/api-parity.py --by-topic      # what is left, per stage
    scripts/api-parity.py --check         # fail on anything unledgered
    scripts/api-parity.py --check-ported  # ...including the compat-shim surface
    scripts/api-parity.py --check --require-disposition   # RFC-0087's four
    scripts/api-parity.py --refresh       # re-derive the ROS 2 side from source
    scripts/api-parity.py --self-test

# Where the ROS 2 side comes from

Re-derived by `--refresh` from a real installation and CACHED under
`docs/reference/api-surface/`, for the reason `scripts/rmw-api-parity.py` caches
its contract: the comparison must be runnable on a host with no ROS, no rclc
checkout and no rclrs workspace, or it runs on one host and rots everywhere else.

  rclcpp  `/opt/ros/<distro>/include` -- installed headers, parsed by clang.
  rclc    a git checkout of `ros2/rclc` (`--rclc <path>`); it is not part of a
          desktop ROS install, being a micro-ROS package. Compared TOGETHER WITH
          `rcl`, because rclc is a convenience layer and not a whole API: its
          own examples call `rcl_publish`, `rcl_take` and `rcl_*_fini` directly
          (23 `rclc_executor_init` against 6 `rcl_publish` in `rclc_examples`).
          Comparing against rclc alone would score our publish and take entry
          points as inventions when they are the ROS 2 C API doing its job.
  rclrs   a `ros2_rust` workspace checkout (`--rclrs <path>`); not packaged at all.

# Only PUBLIC ROS 2 items are compared

nano-ros aligns to the API a ROS 2 user writes, not to rclcpp's callback type
erasure, rcl's wait-set plumbing, or the generated accessors of
`rcl_interfaces`. `public_surface.py` drops those, keyed on the file each
declaration came from rather than on its name, and the report says how many
each tier removed. `--include-internal` compares everything.

# What a verdict means

`--check` demands that every non-matching row carry a ledger entry, one of:

  divergence  we changed it, and a platform constraint is why. This is the only
              sanctioned reason to differ (see RFC-0036), so the `why` must name
              the constraint -- `no_std`, no exceptions, no allocator, no
              runtime env, single-threaded transport -- not a preference.
  extension   we add it, and an RTOS scenario needs it. ROS 2 has no equivalent.
  declined    ROS 2 has it, we deliberately do not, with the reason.
  gap         ROS 2 has it, we should too, and nobody has done it. A gap is a
              legitimate ledger entry -- the point is that it is WRITTEN DOWN,
              not that it is absent.
  rename      the two names differ and ours is the one that should change. This
              is the campaign's work list: a rename with no platform reason is
              a defect, because the drop-in claim is what it costs.
  their-rename
              the two names differ, the CAPABILITY matches, and THEIRS is the
              outlier: ours is the spelling the broader ROS 2 ecosystem already
              uses. The mirror of `rename`, and the only verdict that says a
              difference is upstream's to close. A row must carry a
              `their_rename` object naming `ours`, the `majority` it agrees
              with (at least one entry citing an upstream: rcl / rclcpp /
              rclrs / rclc / an interface package) and the `outlier`.

The gate is not "no differences". It is "no UNEXPLAINED differences".

# What a DISPOSITION means (RFC-0087, phase-417 W0.b)

A verdict says why WE differ. A disposition says what a PORTING USER GETS:
`adopt`, `adopt-bounded`, `refuse-loud`, `absent`. It is optional today and
validated when present; `--require-disposition` gates `declined` rows on it,
and phase-417 W-M2 is the pass that makes it the default. See DISPOSITIONS.

# The C++ lane measures TWO surfaces (issue 1020, phase-417 W0.a)

`rclcpp_compat.hpp` is what a ported file actually reaches, so it is the fourth
C++ translation unit and namespace `rclcpp` is admitted for it. But the shim's
rows answer a DIFFERENT question from the native `nros::` headers' rows, so
they are not merged into one number: every row carries `surface` and
`native_bucket`, and the report prints both summaries. See CPP_TRANSLATION_UNITS
and `run_lang`.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import fnmatch
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "api_parity"))

import correlate  # noqa: E402
import extract_cxx  # noqa: E402
import extract_rust  # noqa: E402
import public_surface  # noqa: E402
import signature_rules  # noqa: E402
import topics  # noqa: E402

SURFACE_DIR = os.path.join(ROOT, "docs", "reference", "api-surface")
LEDGER_DIR = os.path.join(ROOT, "docs", "reference", "api-parity-ledger")

VERDICTS = ("divergence", "extension", "declined", "gap", "rename", "their-rename")

# `their-rename` is the only verdict that claims the DEFECT IS UPSTREAM'S, so it
# is the only one that can be self-serving. It therefore has to name its
# evidence in a machine-checkable shape rather than in prose a reader has to
# take on trust: which spelling is ours, which ROS 2 spellings ours agrees with,
# and which single upstream spelling is the odd one out. `validate_ledger`
# rejects the verdict without it, and rejects the object without the verdict.
THEIR_RENAME_FIELDS = ("ours", "majority", "outlier")

# At least one `majority` entry must cite an upstream. "our own C and C++ agree"
# is internal consistency, which is a preference -- the whole point of the
# verdict is that ROS 2 ITSELF is on our side.
UPSTREAM_TOKENS = ("rcl", "rclcpp", "rclrs", "rclc", "rcl_interfaces",
                   "action_msgs", "lifecycle_msgs", "rosidl", "rmw", "REP-")

# RFC-0087's four dispositions. A verdict says WHY we differ; a disposition says
# what a PORTING USER GETS, which is the only half a porting user can act on:
#
#   adopt          same name, same observable contract.
#   adopt-bounded  same name and contract, weaker inside an envelope that the
#                  doc comment states. The envelope is part of the API.
#   refuse-loud    we cannot have the contract, and the name is common enough
#                  that a user will reach for it -- so the name EXISTS as a
#                  deleted overload or a `static_assert` naming the constraint
#                  and the nano-ros alternative. Never compile and differ.
#   absent         the name does not exist. Correct for rclcpp internals a user
#                  program never names.
#
# OPTIONAL for now, deliberately. Phase-417 W-M2 is the pass that classifies
# every `declined` row, and `--require-disposition` is the gate it turns on;
# making it mandatory before that pass runs would fail the tree on ~700 rows
# nobody has been asked to classify yet. What IS enforced today is that a
# disposition, if written, is one of these four -- a typo that silently
# satisfies a future gate is the failure a ledger exists to prevent.
DISPOSITIONS = ("adopt", "adopt-bounded", "refuse-loud", "absent")

BUCKETS = ("systematic", "arity-only", "differs", "ours-only", "theirs-only")

LANGS = ("c", "cpp", "rust")

# `rclcpp.hpp` is the header a user includes; the action and lifecycle surfaces
# live in sibling packages that a ROS 2 user also includes by name.
RCLCPP_SOURCE = (
    "#include <rclcpp/rclcpp.hpp>\n"
    "#include <rclcpp_action/rclcpp_action.hpp>\n"
    "#include <rclcpp_lifecycle/lifecycle_node.hpp>\n"
)
RCLCPP_NAMESPACES = {"rclcpp", "rclcpp_action", "rclcpp_lifecycle"}

RCLC_SOURCE = (
    "#include <rcl/rcl.h>\n"
    "#include <rcl_action/rcl_action.h>\n"
    "#include <rcl/graph.h>\n"
    "#include <rcl/logging.h>\n"
    "#include <rclc/rclc.h>\n"
    "#include <rclc/executor.h>\n"
    "#include <rclc/action_server.h>\n"
    "#include <rclc/action_client.h>\n"
    "#include <rclc_lifecycle/rclc_lifecycle.h>\n"
    "#include <rclc_parameter/rclc_parameter.h>\n"
)


# --------------------------------------------------------------------------
# our side -- always extracted live, never cached
#
# Caching OUR surface would defeat the tool: the whole point is to notice when
# an edit to our headers moves us away from ROS 2, and a cache would report the
# surface as it was when somebody last remembered to refresh it.
# --------------------------------------------------------------------------


# issue 0818 — the C++ surface is the UNION of three translation units, not
# whatever `nros.hpp` happens to reach.
#
# `nros.hpp` is a curated convenience header. Using it as the definition of "our
# C++ API" silently made every header it omits non-API, and the tool reported
# that silence as agreement:
#
#   * `component_node.hpp` is not included by it at all, so `ComponentNode`,
#     `NodeHandle`, the `bind_*` family and the `create_*_raw` family produced
#     ZERO rows while holding ~half the C++ `create_timer` call sites.
#   * `std_compat.hpp` IS included, but behind `#ifdef NROS_CPP_STD`, which this
#     extractor never defined — so its eleven free functions were invisible.
#
# Two wrong ledger rows came out of that in one day: W5 group A renamed
# `make_publisher` -> `create_publisher` against a row recording no collision
# when `nros::create_publisher` already existed in std_compat, and
# `cpp:create_timer` claimed the fix was "ADDING a free function" that had
# existed all along.
#
# The std flavour is extracted SEPARATELY rather than by defining the macro for
# everything, because a `no_std` consumer genuinely does not get those symbols —
# folding them into the base surface would trade one wrong answer for another.
# Items reachable only with the flag are tagged `std_only` so a row can say so.
#
# issue 1020 — and the FOURTH TU is `rclcpp_compat.hpp`, which is issue 0818 one
# level up. The argument the comment above makes for `component_node.hpp` was
# never made for the compat shim, and it is the stronger case: 589 lines whose
# entire purpose is drop-in compatibility, and what a ported file actually
# reaches, because `#include <rclcpp/rclcpp.hpp>` resolves to it through
# `cmake/compat/NrosRclcppCompat.cmake`. Excluded TWICE until now — the header
# was never included, and everything it declares is in namespace `rclcpp`, which
# the `{"nros"}` filter would have dropped anyway. So the 717 uncovered rclcpp
# rows measured how far the NATIVE `nros::` API is from rclcpp, not how far
# nano-ros is, and that was the headline number.
#
# Hence the per-TU namespace roots: the shim admits `rclcpp`, and the native TUs
# stay on `nros` so a native row cannot silently become a shim row.
#
# `surface` is the answer to issue 1020's second question — see SURFACE_* below.
NATIVE, PORTED = correlate.NATIVE, correlate.PORTED

CPP_TRANSLATION_UNITS = (
    # label, source, extra clang args, namespace roots, marks on every record
    ("base", '#include "nros/nros.hpp"\n', (), {"nros"}, {}),
    ("component", '#include "nros/component_node.hpp"\n', (), {"nros"}, {}),
    ("std", '#include "nros/nros.hpp"\n', ("-DNROS_CPP_STD=1",), {"nros"},
     {"std_only": True}),
    # The PORTED surface — what a file including <rclcpp/rclcpp.hpp> reaches.
    #
    # It used to be `nros/rclcpp_compat.hpp`, a shim declaring the `rclcpp::`
    # names over ours. phase-417 stage 6 step A moved every one of those
    # declarations into the header that owns the concept and DELETED the shim,
    # so the source here is the umbrella — the same text as the `base` TU, read
    # for a different namespace. That is the whole point of the step: there is
    # one set of headers and two vocabularies, not two sets of headers.
    #
    # Still `std_only`, and the MEASURED reason is not the one issue 1020
    # states. The issue says the shim was "reachable only under NROS_CPP_STD";
    # it was never macro-gated — extracting with and without the flag yielded
    # identical records. What made it std-only was unconditional `<memory>`,
    # `<string>`, `<functional>`, `<vector>`, `<chrono>` and `std::shared_ptr`
    # in public signatures. After the move those includes are `__has_include`-
    # gated, so the `rclcpp::` names that need them are genuinely absent from a
    # freestanding build — which makes the marking TRUE now in the way the issue
    # only claimed it was.
    #
    # Extracted WITH the flag, because that is the flavour the compat CMake path
    # builds under, and a surface should be measured as it ships.
    ("compat", '#include "nros/nros.hpp"\n', ("-DNROS_CPP_STD=1",),
     {"rclcpp", "rclcpp_action", "rclcpp_lifecycle"},
     {"std_only": True, "surface": PORTED}),
)


# issue 1020, decision 2: TWO surfaces, one lane, one ledger.
#
# "How close is the NATIVE API to rclcpp" and "what does a PORTED FILE hit" are
# different questions with different answers, and the lane answered only the
# first. Merging them silently would answer neither: measured, the shim moves
# five rows OFF a real classification — `Node::create_publisher`,
# `Node::create_subscription` and `spin` go `systematic` -> `same`, `Node::Node`
# and `init` go `arity-only` -> `same` — because `compare` merges the shim's
# overload into the native item's key and `arity_verdict` reports agreement if
# ANY overload pair agrees. The native answer for those rows would simply cease
# to exist.
#
# A separate LANE was the alternative and is worse: it doubles the clang run,
# and it forks the ledger, so a row would need classifying twice and the two
# copies would drift. Instead every row carries `surface` (which of the two it
# is reachable from) and `native_bucket` (what the correlator said before the
# shim was admitted), and the report prints both summaries. One extraction, one
# ledger, two answers.
SURFACE_NOTE = {
    NATIVE: "reachable from the nros:: headers",
    PORTED: "only via rclcpp_compat.hpp",
}


def ours_cpp(tmpdir):
    # The de-dup key is the WHOLE RECORD, not its `qual`. `extract` emits one
    # record per DECLARATION -- overloads are separate records that `correlate.
    # flatten` groups afterwards, and a type's record carries its member list --
    # so collapsing on the name alone drops exactly what the extra TUs were
    # added to see. Measured: `nros::spin` lost its `(uint32_t, int32_t)`
    # overload and was reported `differs` against rclcpp when it is
    # `systematic`; `nros::init` lost three of four; `Timer::attach_std_closure`,
    # `GuardCondition::attach_std_closure` and `Seq::to_vector` vanished because
    # their owning type was already seen in the base TU with a shorter member
    # list. That is issue 0818's own failure -- a silently narrowed C++ surface
    # reported as agreement -- one level down.
    seen = set()
    order = []
    for _label, source, extra, roots, marks in CPP_TRANSLATION_UNITS:
        items = extract_cxx.extract(
            source,
            "c++",
            extract_cxx.nros_cpp_include_args() + list(extra),
            roots,
            tmpdir,
        )
        for item in items:
            key = json.dumps(item, sort_keys=True)
            if key in seen:
                continue
            seen.add(key)
            # Marks are applied AFTER the de-dup test, never before: they would
            # otherwise be part of the key, and a record emitted identically by
            # two TUs would stop de-duplicating. `std_only` has always been set
            # here for that reason; `surface` joins it.
            item.update(marks)
            order.append(item)
    return order


def ours_c(tmpdir):
    return extract_cxx.extract(
        '#include "nros/nros.h"\n',
        "c",
        extract_cxx.nros_c_include_args(),
        {""},
        tmpdir,
        prefixes={"nros_", "NROS_"},
    )


def ours_rust(tmpdir):
    docdir, _ = extract_rust.rustdoc_json(
        os.path.join(ROOT, "packages", "api", "nros"),
        with_deps=True,
        target_dir=os.path.join(tmpdir, "rustdoc-ours"),
        features=extract_rust.NROS_FEATURES,
    )
    docs = extract_rust.Docs(docdir)
    return extract_rust.surface(docs, docs.crate("nros"), "nros")


# --------------------------------------------------------------------------
# their side -- cached, re-derivable
# --------------------------------------------------------------------------


def theirs_path(lang):
    return os.path.join(SURFACE_DIR, {"c": "rclc", "cpp": "rclcpp", "rust": "rclrs"}[lang] + ".json")


def load_theirs(lang):
    path = theirs_path(lang)
    if not os.path.exists(path):
        raise SystemExit(
            "no recorded surface at %s.\nRun `scripts/api-parity.py --refresh --lang %s` "
            "on a host that has the source (see this script's docstring)." % (path, lang)
        )
    with open(path) as fh:
        return json.load(fh)


def derive_rclcpp(prefix, tmpdir):
    return extract_cxx.extract(
        RCLCPP_SOURCE,
        "c++",
        extract_cxx.ros_include_args(prefix),
        RCLCPP_NAMESPACES,
        tmpdir,
    )


def derive_rclc(prefix, rclc_root, tmpdir):
    inc = extract_cxx.ros_include_args(prefix)
    for pkg in ("rclc", "rclc_lifecycle", "rclc_parameter"):
        inc.append("-I" + os.path.join(rclc_root, pkg, "include"))
    return extract_cxx.extract(
        RCLC_SOURCE, "c", inc, {""}, tmpdir, prefixes={"rclc_", "RCLC_", "rcl_", "RCL_"}
    )


def derive_rclrs(rclrs_root, tmpdir):
    docdir, _ = extract_rust.rustdoc_json(
        rclrs_root, with_deps=False, target_dir=os.path.join(tmpdir, "rustdoc-rclrs")
    )
    docs = extract_rust.Docs(docdir)
    return extract_rust.surface(docs, docs.crate("rclrs"), "rclrs")


def provenance(lang, prefix, rclc_root, rclrs_root):
    """Record WHAT a cached surface came from, so a stale one can be spotted.

    Deliberately NOT the local path it was derived from. These files are
    committed, and a checkout directory on the machine that ran `--refresh`
    identifies nothing to anyone else -- it is noise at best and a leaked home
    directory at worst. A distro name, a crate version and a git ref are what
    another host can actually compare against.
    """
    if lang == "cpp":
        return {"package": "rclcpp", "distro": os.path.basename(prefix.rstrip("/"))}
    if lang == "c":
        return {
            "package": "rclc+rcl",
            "distro": os.path.basename(prefix.rstrip("/")),
            "rclc_ref": _git_describe(rclc_root),
        }
    return {
        "package": "rclrs",
        "version": _cargo_version(rclrs_root),
        "ref": _git_describe(rclrs_root),
    }


def _git_describe(path):
    try:
        out = subprocess.run(
            ["git", "-C", path, "rev-parse", "HEAD"], capture_output=True, text=True
        )
        return out.stdout.strip() or None
    except OSError:
        return None


def _cargo_version(path):
    manifest = os.path.join(path, "Cargo.toml")
    if not os.path.exists(manifest):
        return None
    for line in open(manifest):
        if line.startswith("version"):
            return line.split("=", 1)[1].strip().strip('"')
    return None


def refresh(langs, prefix, rclc_root, rclrs_root):
    os.makedirs(SURFACE_DIR, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmpdir:
        for lang in langs:
            if lang == "cpp":
                recs = derive_rclcpp(prefix, tmpdir)
            elif lang == "c":
                if not rclc_root:
                    raise SystemExit("--rclc <path to a ros2/rclc checkout> is required for --lang c")
                recs = derive_rclc(prefix, rclc_root, tmpdir)
            else:
                if not rclrs_root:
                    raise SystemExit("--rclrs <path to the rclrs crate> is required for --lang rust")
                recs = derive_rclrs(rclrs_root, tmpdir)
            payload = {
                "provenance": provenance(lang, prefix, rclc_root, rclrs_root),
                "records": recs,
            }
            with open(theirs_path(lang), "w") as fh:
                json.dump(payload, fh, indent=1, sort_keys=True)
                fh.write("\n")
            print("recorded %d records -> %s" % (len(recs), os.path.relpath(theirs_path(lang), ROOT)))


# --------------------------------------------------------------------------
# ledger
# --------------------------------------------------------------------------


def load_ledger():
    """Every ledger shard, merged, minus their documentation.

    One file per language. The split is for CONCURRENCY, not taste: classifying
    ~1300 rows is work for several people at once, and a single file makes every
    one of them rebase against the others. A shard is only ever touched by
    whoever owns that lane.

    JSON has no comments, so each shard explains itself in `_doc`. Keys
    beginning with `_` are documentation and are never entries.

    A shard is named after a TOPIC -- `node.json`, `pubsub.json` -- and holds
    that topic's rows in ALL THREE languages. The campaign closes a feature at a
    time across every language, so the topic is what an agent owns and what
    "complete" is asserted about. Sharding by language instead would let C++
    pubsub land while C pubsub sits unexamined, and the drop-in claim is made
    per language: a feature that works in one is not a feature.

    A row filed in the wrong shard is a real error, not a harmless one -- the
    shard is the topic's inventory, and a `qos` row hiding in `node.json` makes
    both stages' counts wrong. `--self-test` rejects it, using the same
    `topics.topic_of` the report groups by, so a shard cannot disagree with the
    taxonomy.
    """
    merged = {}
    if not os.path.isdir(LEDGER_DIR):
        return merged
    for name in sorted(os.listdir(LEDGER_DIR)):
        if not name.endswith(".json"):
            continue
        shard_topic = name[: -len(".json")]
        with open(os.path.join(LEDGER_DIR, name)) as fh:
            text = fh.read()
        # A DUPLICATE KEY is silently survivable and therefore dangerous: JSON
        # parsers keep the last occurrence, so a second row for the same symbol
        # discards the first author's verdict and reasoning with no error
        # anywhere. It happens because two sessions classify the same symbol
        # independently and git merges both -- they land in different regions of
        # the file, so there is no textual conflict to notice. `other.json`
        # carried `cpp:declared_depth` twice on 2026-09-04 for exactly that
        # reason; both said `extension`, which is luck, not a guarantee.
        #
        # The hook runs on EVERY object, nested ones included, and returns a
        # real dict -- an earlier version returned the pair list itself, which
        # detected the duplicates but left every nested object (`rename`,
        # `their_rename`) as a list of tuples for the rest of the tool. Nothing
        # read below the top level at the time, so the trap only sprang when a
        # validator finally did.
        def no_duplicate_keys(pairs, _name=name):
            seen = set()
            for key, _ in pairs:
                if key in seen:
                    raise SystemExit(
                        f"api-parity: {_name} defines {key!r} more than once. JSON keeps "
                        f"the LAST, so the other row's verdict is being discarded "
                        f"silently. Merge them into one row -- do not delete either "
                        f"reason."
                    )
                seen.add(key)
            return dict(pairs)

        raw = json.loads(text, object_pairs_hook=no_duplicate_keys)
        for key, value in raw.items():
            if key.startswith("_"):
                continue
            value = dict(value)
            value["_shard"] = shard_topic
            # The lane is what correctness turns on; the FILENAME is what the
            # person fixing it has to open. A message naming `cpp.json` when the
            # row sits in `cpp-node.json` sends them to a file that need not
            # exist.
            value["_file"] = name
            merged[key] = value
    return merged


def ledger_key(lang, key):
    return "%s:%s" % (lang, key)


def lookup(ledger, lang, key, bucket, buckets_by_key):
    """(entry, inherited) for a row -- a member may inherit its TYPE's verdict.

    `rclcpp::Node` has 49 public methods we do not have. Writing 49 sentences
    that each say "we have no Node" is not a ledger, it is a copy-paste
    exercise, and the fiftieth reader stops reading. So a row on `Node` covers
    `Node::*`.

    The inheritance is conditional, and the condition is the point: it applies
    only when the TYPE is in the SAME bucket as the member. If we have `Node`
    but not `Node::declare_parameter`, the type is `same` and the method is
    `theirs-only` -- a real gap in a type we ship, which is a different
    statement from "we do not have this type" and must be argued on its own.
    An inherited verdict prints with a trailing `*`.
    """
    own = ledger.get(ledger_key(lang, key))
    if own is not None:
        return own, False

    if "::" in key:
        owner = key.rsplit("::", 1)[0]
        if buckets_by_key.get(owner) == bucket:
            inherited = ledger.get(ledger_key(lang, owner))
            if inherited is not None:
                return inherited, True

    # A glob row covers a family. The C surface needs this and the C++/Rust
    # surfaces do not: C names are flat, so `publisher_init` and
    # `publisher_fini` share no owning type for a verdict to descend from, and
    # the entity prefix is the only structure the API has.
    #
    # A glob must DECLARE the bucket it covers, and only matches rows in it.
    # Without that, one `c:action_*` row would silently absorb a gap, an
    # extension and an unexplained signature change alike -- three different
    # claims under one sentence, which is the failure a ledger exists to
    # prevent. Most specific glob wins, so a narrower row can override.
    best = None
    for lkey, entry in ledger.items():
        klang, _, pattern = lkey.partition(":")
        if klang != lang or "*" not in pattern:
            continue
        if entry.get("bucket") != bucket:
            continue
        if not fnmatch.fnmatchcase(key, pattern):
            continue
        if best is None or len(pattern) > len(best[0]):
            best = (pattern, entry)
    if best is not None:
        return best[1], True
    return None, False


def validate_their_rename(key, value):
    """The evidence a `their-rename` row owes, and nobody else may claim.

    Kept separate so the self-test can drive it with rows that are wrong in one
    way each -- a validator exercised only through the good ledger proves the
    good ledger is good and nothing about the validator.
    """
    problems = []
    evidence = value.get("their_rename")
    is_verdict = value.get("verdict") == "their-rename"

    if evidence is None:
        if is_verdict:
            problems.append(
                "ledger %s: verdict `their-rename` needs a `their_rename` object "
                "naming %s" % (key, ", ".join(THEIR_RENAME_FIELDS))
            )
        return problems

    if not is_verdict:
        problems.append(
            "ledger %s: carries `their_rename` evidence under verdict %r -- the "
            "object is the claim that THEIRS is the outlier, so it belongs only "
            "on `their-rename`" % (key, value.get("verdict"))
        )
    if not isinstance(evidence, dict):
        problems.append("ledger %s: `their_rename` must be an object" % key)
        return problems

    for field in THEIR_RENAME_FIELDS:
        if not evidence.get(field):
            problems.append("ledger %s: `their_rename` is missing %r" % (key, field))
    majority = evidence.get("majority")
    if majority is not None and not isinstance(majority, list):
        problems.append("ledger %s: `their_rename.majority` must be a list" % key)
    elif majority:
        if not any(
            isinstance(m, str) and any(t in m for t in UPSTREAM_TOKENS) for m in majority
        ):
            problems.append(
                "ledger %s: `their_rename.majority` cites no upstream (%s) -- our "
                "own languages agreeing is internal consistency, not the ROS 2 "
                "ecosystem" % (key, "/".join(UPSTREAM_TOKENS[:5]))
            )
    return problems


def _validate_provides_targets(entries, surfaces):
    """Every UPSTREAM-shaped `provides` target must resolve in a recorded surface.

    phase-417 stage 6. Two rows carried `provides: ["rclrs::log_trace"]` and
    rclrs's severity family stops at `debug` -- the arrows pointed at nothing.
    They were generated MECHANICALLY as `rclrs::log_<severity>` from the macro
    name, which is the one thing the original `provides` pass avoided: it
    hand-wrote every arrow, and two independent authors rejected ~76% of
    mechanically proposed candidates as noise.

    Only upstream-shaped targets are checked. An arrow may legitimately name one
    of OUR symbols (most do -- 133 of 143), and this file has no view of our
    surface without running the extractors, which would make a cheap structural
    check depend on clang and a nightly rustdoc.
    """
    known = set()
    for payload in surfaces.values():
        for record in payload.get("records", []):
            qual = record.get("qual")
            if qual:
                known.add(qual)
    if not known:
        return []
    complaints = []
    for key, value in entries.items():
        if key == "_doc" or not isinstance(value, dict):
            continue
        for target in value.get("provides", []) or []:
            upstream = target.startswith(("rclrs::", "rclcpp::", "rcl_", "rclc_"))
            if upstream and target not in known:
                complaints.append(
                    "ledger %s: `provides` names %s, which no recorded surface declares. "
                    "An arrow that resolves to nothing reads as a re-map and is one."
                    % (key, target)
                )
    return complaints


def validate_ledger(entries):
    """Structural complaints about ledger rows -- what is checkable with no build.

    Separated from `load_ledger` so the self-test can feed it a bad row. A
    validator that can only be run against the good ledger on disk proves the
    good ledger is good and nothing about the check.

    Deliberately does NOT check that a row sits in the right topic shard. A
    topic is decided by the DECLARING HEADER (see `topics.topic_of`), and a
    ledger row carries no header -- deciding it from the name alone here would
    make the gate disagree with the report for exactly the rows the header was
    introduced to get right. `--check` does that check, where the header is
    known.
    """
    problems = []
    for key, value in sorted(entries.items()):
        if value.get("verdict") not in VERDICTS:
            problems.append("ledger %s: unknown verdict %r" % (key, value.get("verdict")))
        if not value.get("why", "").strip():
            problems.append("ledger %s: empty reason" % key)
        # `disposition` is OPTIONAL (see DISPOSITIONS) -- but a value that is
        # present must be one of the four. An unrecognised one would satisfy
        # `--require-disposition` when W-M2 turns it on while saying nothing,
        # which is the exact shape of the typo'd-verdict failure above.
        if "disposition" in value and value["disposition"] not in DISPOSITIONS:
            problems.append(
                "ledger %s: unknown disposition %r (one of %s)"
                % (key, value.get("disposition"), ", ".join(DISPOSITIONS))
            )
        problems.extend(validate_their_rename(key, value))
        pattern = key.partition(":")[2]
        if "*" in pattern and value.get("bucket") not in BUCKETS:
            problems.append(
                "ledger %s: a glob row must declare the bucket it covers (one of %s)"
                % (key, ", ".join(BUCKETS))
            )
        lang = key.split(":", 1)[0]
        if lang not in LANGS:
            problems.append("ledger %s: unknown language %r" % (key, lang))

        if "_shard" in value and value["_shard"] not in topics.NAMES:
            problems.append(
                "ledger %s: %s is not a topic; shards are named for one of %s"
                % (key, value.get("_file", "?"), ", ".join(topics.NAMES))
            )
    return problems


def undisposed(entries):
    """`declined` rows that do not say what a porting user GETS.

    RFC-0087's consequence for RFC-0036: a `declined` verdict with no
    disposition does not say whether a ported program gets a compile error or a
    surprise, and that is the only thing a porting user needs to know.

    NOT wired into `--check`, on purpose, and phase-417 W-M2 is the work item
    that wires it. Two reasons it waits. The classification pass has not run --
    all ~700 declines would fail at once, so the gate would be turned off again
    the same day it landed. And the ledger shards are being edited concurrently
    by the correction track (issues 1012, 1022); a field that is required
    before anyone has been asked to write it breaks whoever is mid-edit.

    `--require-disposition` opts in early, so the pass can gate itself as it
    goes rather than in one flip at the end.
    """
    return sorted(
        key for key, value in entries.items()
        if value.get("verdict") == "declined" and not value.get("disposition")
    )


# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------


def run_lang(lang, tmpdir, include_internal=False):
    """Rows for one language, each annotated with WHICH SURFACE it belongs to.

    Every row gains two fields beyond the correlator's own:

      `surface`        `native`, `ported`, or "" for a row we do not ship at
                       all. It is the OURS item's mark, so a key that both a
                       native header and the compat shim declare reads `native`
                       -- the shim is the second way to reach it, not the only
                       one.
      `native_bucket`  the bucket the correlator assigns with the compat shim's
                       records removed, or None if the key does not exist
                       natively. This is RE-CORRELATED, not derived from the
                       merged rows, because the merge is lossy in both
                       directions: the shim can turn `systematic` into `same`
                       by adding an agreeing overload, and it can turn a
                       `theirs-only` TYPE into `same` and thereby stop its
                       members inheriting that type's ledger verdict.

    The second run is pure Python over ~1200 keys; the expensive half (clang,
    rustdoc) has already happened by then, so two answers cost one extraction.
    """
    ours_records = {"c": ours_c, "cpp": ours_cpp, "rust": ours_rust}[lang](tmpdir)
    payload = load_theirs(lang)
    theirs_records = payload["records"]
    removed = {}
    if not include_internal:
        # Filtered at REPORT time, not at --refresh time: the recorded surface
        # stays complete, so tightening or loosening the public-API rule is a
        # code change rather than a re-derivation on a host with ROS installed.
        theirs_records, removed = public_surface.filter_records(theirs_records)
    clang = {"c": "c", "cpp": "c++", "rust": "rust"}[lang]
    theirs = correlate.flatten(theirs_records, clang, "theirs")
    rows = correlate.compare(correlate.flatten(ours_records, clang, "ours"),
                             theirs, clang)

    native_records = [r for r in ours_records if r.get("surface") != PORTED]
    if len(native_records) == len(ours_records):
        native = {r["key"]: r["bucket"] for r in rows}
    else:
        native = {
            r["key"]: r["bucket"]
            for r in correlate.compare(
                correlate.flatten(native_records, clang, "ours"), theirs, clang)
        }
    for row in rows:
        item = row.get("ours")
        row["surface"] = (item.get("surface") or NATIVE) if item else ""
        row["native_bucket"] = native.get(row["key"])
    return rows, payload.get("provenance", {}), removed


def surface_counts(rows):
    """(ported_counts, native_counts) -- the two answers, side by side.

    A row absent from the native surface contributes to `native` only when it
    still exists there as a `theirs-only` statement about upstream; a shim-only
    `ours-only` row has no native counterpart at all and is simply not counted.
    """
    ported, native = {}, {}
    for r in rows:
        ported[r["bucket"]] = ported.get(r["bucket"], 0) + 1
        nb = r.get("native_bucket")
        if nb:
            native[nb] = native.get(nb, 0) + 1
    return ported, native


def row_topic(row):
    """The stage a report row belongs to.

    Prefers THEIRS's header, because a `theirs-only` row is a statement about
    the ROS 2 surface and that is the surface being carved into stages. Falls
    back to ours, then to the name alone.
    """
    header = ""
    for side in ("theirs", "ours"):
        item = row.get(side)
        if item and item.get("header"):
            header = item["header"]
            break
    return topics.topic_of(row["key"], header)


def by_topic(langs, tmpdir):
    """Per stage, per language, how many DECISIONS are still unledgered.

    A decision, not a row: a member whose type carries the verdict is already
    answered, so counting rows would report the same work several times and make
    a finished stage look unfinished.
    """
    ledger = load_ledger()
    table = {}
    for lang in langs:
        rows, _prov, _removed = run_lang(lang, tmpdir)
        buckets = {r["key"]: r["bucket"] for r in rows}
        for r in rows:
            if r["bucket"] == "same":
                continue
            entry, _inh = lookup(ledger, lang, r["key"], r["bucket"], buckets)
            if entry is not None or r["bucket"] == "systematic":
                continue
            if "::" in r["key"]:
                owner = r["key"].rsplit("::", 1)[0]
                if buckets.get(owner) == r["bucket"]:
                    # Answered by whatever answers its type.
                    continue
            topic = row_topic(r)
            table.setdefault(topic, {}).setdefault(lang, 0)
            table[topic][lang] += 1

    print("\ndecisions still open, by stage (in the order STAGE_ORDER gives):\n")
    print("    %-10s %8s %8s %8s %8s" % ("stage", *langs, "total"))
    grand = 0
    for topic in topics.STAGE_ORDER:
        counts = table.get(topic, {})
        total = sum(counts.values())
        grand += total
        if not total:
            print("    %-10s %8s %8s %8s %8s   done" % (topic, *(
                counts.get(l, 0) for l in langs), total))
            continue
        print("    %-10s %8d %8d %8d %8d" % (
            topic, *(counts.get(l, 0) for l in langs), total))
    print("    %-10s %8s %8s %8s %8d" % ("", "", "", "", grand))
    return 0


def _bucket_line(counts):
    return (
        "same %d   arity-only %d   systematic %d   differs %d   "
        "ours-only %d   theirs-only %d"
        % (
            counts.get("same", 0),
            counts.get("arity-only", 0),
            counts.get("systematic", 0),
            counts.get("differs", 0),
            counts.get("ours-only", 0),
            counts.get("theirs-only", 0),
        )
    )


def gate_rows(ledger, lang, rows, key_bucket="bucket"):
    """Every row that differs and carries no ledger entry, on ONE surface.

    `key_bucket` selects which correlation is being gated: `bucket` is the
    ported-file surface, `native_bucket` the native one. The bucket is not a
    cosmetic choice -- `lookup` inherits a type's verdict only when the member
    sits in the SAME bucket as its type, so the two surfaces genuinely disagree
    about which rows are answered.

    Runs over ALL rows, independently of `--grep` / `--topic` / `--show`. The
    gate used to be collected inside the printing loop, so a filtered report
    silently gated a filtered surface.
    """
    buckets = {}
    for r in rows:
        b = r.get(key_bucket)
        if b:
            buckets[r["key"]] = b
    out = []
    for r in rows:
        b = r.get(key_bucket)
        if not b or b in ("same", "systematic"):
            continue
        if lookup(ledger, lang, r["key"], b, buckets)[0] is None:
            out.append((lang, b, r["key"]))
    return out


def report(langs, show, check, suggest, include_internal, grep=None, topic=None,
           check_ported=False, require_disposition=False):
    ledger = load_ledger()
    unledgered = []
    ported_unledgered = []
    misfiled = []
    with tempfile.TemporaryDirectory() as tmpdir:
        for lang in langs:
            rows, prov, removed = run_lang(lang, tmpdir, include_internal)
            ported_counts, native_counts = surface_counts(rows)
            counts = ported_counts
            by_key = {r["key"]: r["bucket"] for r in rows}

            # The gate runs on the NATIVE surface, which is the one the ledger
            # was written against. The ported surface is REPORTED and, for now,
            # not enforced -- see `--check-ported`.
            unledgered.extend(gate_rows(ledger, lang, rows, "native_bucket"))
            ported_only = [
                x for x in gate_rows(ledger, lang, rows, "bucket")
                if x not in set(unledgered)
            ]
            ported_unledgered.extend(ported_only)

            print("\n=== %s vs %s ===" % (lang, prov.get("package", "?")))
            if prov:
                bits = [f"{k}={v}" for k, v in sorted(prov.items()) if v]
                print("    " + "  ".join(bits))
            two_surfaces = native_counts and native_counts != ported_counts
            if two_surfaces:
                # issue 1020: the two questions, answered separately and in the
                # order a reader of the campaign's headline number needs them.
                print("    ported-file surface (what a file including "
                      "<rclcpp/rclcpp.hpp> reaches):")
                print("      " + _bucket_line(ported_counts))
                print("    native API surface (nros:: headers alone, the "
                      "denominator this lane used to report):")
                print("      " + _bucket_line(native_counts))
                moved = sum(1 for r in rows
                            if r.get("native_bucket") and
                            r["native_bucket"] != r["bucket"])
                added = sum(1 for r in rows if r.get("native_bucket") is None)
                print("      the compat shim reclassifies %d row(s) and adds "
                      "%d that do not exist natively" % (moved, added))
            else:
                print("    " + _bucket_line(counts))
            if removed:
                # Never silent: a filter that shrinks a number without saying
                # what it took reads exactly like progress.
                print(
                    "    not public API, excluded: "
                    + "  ".join("%s %d" % (t, n) for t, n in sorted(removed.items()))
                )
            elif include_internal:
                print("    --include-internal: comparing the whole ROS 2 surface")

            for r in rows:
                bucket = r["bucket"]
                if grep is not None and not grep.search(r["key"]):
                    continue
                if topic is not None and row_topic(r) != topic:
                    continue
                if bucket == "same" and "same" not in show:
                    continue
                if bucket != "same" and bucket not in show and show != {"all"}:
                    if show and "all" not in show:
                        continue
                entry, inherited = lookup(ledger, lang, r["key"], bucket, by_key)
                if entry is not None and not inherited:
                    want = row_topic(r)
                    if entry.get("_shard") not in (None, want):
                        misfiled.append((ledger_key(lang, r["key"]),
                                         entry["_shard"], want))
                verdict = (entry["verdict"] + "*") if (entry and inherited) else (
                    entry["verdict"] if entry else ""
                )
                if bucket == "systematic":
                    # The rule IS the explanation. Requiring a ledger row too
                    # would restate one sentence once per site, which is how the
                    # sentence stops being read.
                    verdict = "rule:" + ",".join(r["detail"]["rules"])
                mark = {"same": " ", "arity-only": "?", "systematic": "=",
                        "differs": "!", "ours-only": "+", "theirs-only": "-"}[bucket]
                # The two-surface annotation, per row. `[shim]` says a ported
                # file is the only way to reach our side of this row;
                # `native:X` says the native answer differs from the ported one
                # and states it, so a `same` the shim produced can never be
                # read as the native API having closed the gap.
                note = ""
                if r.get("surface") == PORTED:
                    note += "  [shim]"
                nb = r.get("native_bucket")
                if nb and nb != bucket:
                    note += "  native:%s" % nb
                disp = (entry or {}).get("disposition")
                if disp:
                    note += "  <%s>" % disp
                line = "  %s %-52s %-12s %s%s" % (
                    mark, r["key"], verdict or "UNLEDGERED", bucket, note)
                print(line)
                if bucket in ("differs", "systematic", "arity-only") and r.get("detail"):
                    print(
                        "      ours   %s\n      theirs %s"
                        % (
                            correlate.render_params(r["ours"]),
                            correlate.render_params(r["theirs"]),
                        )
                    )

            if suggest:
                pairs = correlate.suggest_renames(rows)
                print(
                    "\n  possible renames (%d) -- SIMILARITY, not evidence; confirm each\n"
                    "  before writing a ledger row, and expect false pairs:" % len(pairs)
                )
                for ours_key, theirs_key, ratio in pairs:
                    print("    %.2f  %-42s ->  %s" % (ratio, ours_key, theirs_key))

    def _list(rows_):
        for lang_, bucket_, key_ in rows_[:40]:
            print("  %s:%s  (%s)" % (lang_, key_, bucket_), file=sys.stderr)
        if len(rows_) > 40:
            print("  ... and %d more" % (len(rows_) - 40), file=sys.stderr)

    if ported_unledgered:
        # Printed ALWAYS, gated only under `--check-ported`. These are the rows
        # the compat shim opened up and nobody has classified, and the whole
        # point of stage 0 is that the number stops being invisible. Most are
        # the same mechanism: the shim ships `NodeOptions`, `Logger` and
        # `Server` as names, so those types stop being `theirs-only`, so their
        # members stop inheriting "we do not have this type" -- and a gap INSIDE
        # a type we ship is a different claim that has to be argued on its own.
        print(
            "\n%d item(s) on the PORTED-FILE surface have no ledger entry "
            "(reported, not gated -- phase-417 W-M2 classifies them; "
            "--check-ported gates them now):" % len(ported_unledgered),
            file=sys.stderr,
        )
        _list(ported_unledgered)

    if check:
        if misfiled:
            print(
                "\n%d ledger row(s) sit in the wrong topic shard "
                "(all listed; the move is mechanical):" % len(misfiled),
                file=sys.stderr,
            )
            for key, was, want in misfiled:
                print("  %s  is in %s.json, belongs in %s.json" % (key, was, want),
                      file=sys.stderr)
            return 1
        if require_disposition:
            missing = undisposed(ledger)
            if missing:
                print(
                    "\n%d `declined` ledger row(s) carry no disposition. RFC-0087: "
                    "a decline that does not say what a porting user GETS is not "
                    "actionable. Add `\"disposition\"` -- one of: %s"
                    % (len(missing), ", ".join(DISPOSITIONS)),
                    file=sys.stderr,
                )
                for key in missing[:40]:
                    print("  " + key, file=sys.stderr)
                if len(missing) > 40:
                    print("  ... and %d more" % (len(missing) - 40), file=sys.stderr)
                return 1
        failing = list(unledgered) + (ported_unledgered if check_ported else [])
        if failing:
            print(
                "\n%d item(s) differ with no ledger entry. Add a row to "
                "%s/<lang>.json\nwith one of: %s"
                % (len(failing), os.path.relpath(LEDGER_DIR, ROOT), ", ".join(VERDICTS)),
                file=sys.stderr,
            )
            _list(failing)
            return 1
        print("\nevery divergence carries a ledger entry")
    return 0


def self_test():
    """Exercise the correlator on hand-built records, not on the real surfaces.

    The real surfaces need clang, a ROS install and a nightly toolchain; a
    self-test that needs those is a self-test that gets skipped.
    """
    failures = []

    def check(name, got, want):
        if got != want:
            failures.append("%s: got %r want %r" % (name, got, want))

    check("c prefix ours", correlate.normalize("c", "ours", "nros_publisher_init", "function"), "publisher_init")
    check("c prefix theirs", correlate.normalize("c", "theirs", "rclc_publisher_init", "function"), "publisher_init")
    check(
        "cpp method",
        correlate.normalize("c++", "ours", "nros::Node::create_publisher", "function"),
        "Node::create_publisher",
    )
    check(
        "rust State fold",
        correlate.normalize("rust", "theirs", "rclrs::NodeState::create_publisher", "function"),
        "Node::create_publisher",
    )
    check(
        "rust State type folds",
        correlate.normalize("rust", "theirs", "rclrs::PublisherState", "type"),
        "Publisher",
    )
    state_rows = correlate.compare(
        correlate.flatten(
            [{"kind": "type", "qual": "nros::Publisher", "name": "Publisher",
              "members": [{"name": "publish", "params": [{"type": "&M"}], "ret": "", "template": []}]}],
            "rust", "ours"),
        correlate.flatten(
            [{"kind": "type", "qual": "rclrs::PublisherState", "name": "PublisherState",
              "members": [{"name": "publish", "params": [{"type": "&M"}], "ret": "", "template": []}]}],
            "rust", "theirs"),
        "rust",
    )
    check(
        "an rclrs State split is not a divergence",
        {r["key"]: r["bucket"] for r in state_rows}.get("Publisher::publish"),
        "same",
    )
    check(
        "rust NodeCtx synonym",
        correlate.normalize("rust", "ours", "nros::node::NodeCtx::create_publisher", "function"),
        "Node::create_publisher",
    )
    check("cpp type", correlate.normalize("c++", "ours", "nros::QoS", "type"), "QoS")
    check(
        "rclcpp Base fold",
        correlate.normalize("c++", "theirs", "rclcpp::PublisherBase::get_topic_name", "function"),
        "Publisher::get_topic_name",
    )
    check(
        "a real *Base type is not folded",
        correlate.normalize("c++", "theirs", "rclcpp::node_interfaces::NodeBase::get_name", "function"),
        "NodeBase::get_name",
    )
    # The type key must fold as well -- member keys are built from it, so
    # folding only the method owner changes nothing at all.
    check(
        "rclcpp Base type folds",
        correlate.normalize("c++", "theirs", "rclcpp::PublisherBase", "type"),
        "Publisher",
    )
    base_rows = correlate.compare(
        correlate.flatten(
            [{"kind": "type", "qual": "nros::Publisher", "name": "Publisher",
              "members": [{"name": "get_topic_name", "params": [], "ret": "", "template": []}]}],
            "c++", "ours"),
        correlate.flatten(
            [{"kind": "type", "qual": "rclcpp::PublisherBase", "name": "PublisherBase",
              "members": [{"name": "get_topic_name", "params": [], "ret": "", "template": []}]}],
            "c++", "theirs"),
        "c++",
    )
    check(
        "an inheritance split is not a divergence",
        {r["key"]: r["bucket"] for r in base_rows}.get("Publisher::get_topic_name"),
        "same",
    )
    check("type noise", correlate.canon_type("const std::string &"), "string&")

    ours = correlate.flatten(
        [
            {
                "kind": "type",
                "qual": "nros::Node",
                "name": "Node",
                "members": [
                    {"name": "create_publisher", "params": [{"type": "const char *"}], "ret": "", "template": []},
                    {"name": "only_ours", "params": [], "ret": "", "template": []},
                ],
            }
        ],
        "c++",
        "ours",
    )
    theirs = correlate.flatten(
        [
            {
                "kind": "type",
                "qual": "rclcpp::Node",
                "name": "Node",
                "members": [
                    {
                        "name": "create_publisher",
                        "params": [{"type": "const std::string &"}, {"type": "const rclcpp::QoS &"}],
                        "ret": "",
                        "template": [],
                    },
                    {"name": "only_theirs", "params": [], "ret": "", "template": []},
                ],
            }
        ],
        "c++",
        "theirs",
    )
    rows = {r["key"]: r["bucket"] for r in correlate.compare(ours, theirs, "c++")}
    check("Node same", rows.get("Node"), "same")

    # A library prefix is not a type difference; a meaning is.
    handles = correlate.shares_only_arity(
        {"overloads": [{"params": [{"type": "struct nros_client_t *"}]}]},
        {"overloads": [{"params": [{"type": "rcl_client_t *"}]}]},
    )
    check("prefixed handles are the same type", handles, False)
    # A defaulted tail must be trimmed before positions are compared, or
    # `spin(int32_t = 10)` against `spin()` reads as arity-only.
    check(
        "a defaulted tail does not make a call arity-only",
        correlate.shares_only_arity(
            {"overloads": [{"params": [{"type": "int32_t", "default": True}]}]},
            {"overloads": [{"params": []}]},
        ),
        False,
    )
    check(
        "a shared arity with no shared position is not `same`",
        correlate.shares_only_arity(
            {"overloads": [{"params": [{"type": "const char *"}, {"type": "uint8_t"}]}]},
            {"overloads": [{"params": [{"type": "int"}, {"type": "char **"}]}]},
        ),
        True,
    )

    # A defaulted parameter must not read as a divergence -- `spin(int32_t = 10)`
    # against `spin()` is the convergence issue 0338 landed on purpose.
    defaulted = correlate.flatten(
        [
            {
                "kind": "type",
                "qual": "nros::Executor",
                "name": "Executor",
                "members": [
                    {
                        "name": "spin",
                        "params": [{"type": "int32_t", "default": True}],
                        "ret": "",
                        "template": [],
                    }
                ],
            }
        ],
        "c++",
        "ours",
    )
    plain = correlate.flatten(
        [
            {
                "kind": "type",
                "qual": "rclcpp::Executor",
                "name": "Executor",
                "members": [{"name": "spin", "params": [], "ret": "", "template": []}],
            }
        ],
        "c++",
        "theirs",
    )
    drows = {r["key"]: r["bucket"] for r in correlate.compare(defaulted, plain, "c++")}
    check("default arg is not a divergence", drows.get("Executor::spin"), "same")

    rename_rows = correlate.compare(
        correlate.flatten(
            [{"kind": "function", "qual": "nros::make_publisher", "name": "make_publisher",
              "params": [], "ret": "", "template": []}],
            "c++", "ours"),
        correlate.flatten(
            [{"kind": "function", "qual": "rclcpp::create_publisher", "name": "create_publisher",
              "params": [], "ret": "", "template": []}],
            "c++", "theirs"),
        "c++",
    )
    pairs = correlate.suggest_renames(rename_rows)
    check("rename suggested", [(a, b) for a, b, _ in pairs], [("make_publisher", "create_publisher")])
    check(
        "unlike names are not paired",
        correlate.suggest_renames(
            correlate.compare(
                correlate.flatten(
                    [{"kind": "function", "qual": "nros::zzz_board_locator", "name": "zzz_board_locator",
                      "params": [], "ret": "", "template": []}],
                    "c++", "ours"),
                correlate.flatten(
                    [{"kind": "function", "qual": "rclcpp::spin", "name": "spin",
                      "params": [], "ret": "", "template": []}],
                    "c++", "theirs"),
                "c++",
            )
        ),
        [],
    )
    check("arity differs", rows.get("Node::create_publisher"), "differs")
    check("ours-only", rows.get("Node::only_ours"), "ours-only")
    check("theirs-only", rows.get("Node::only_theirs"), "theirs-only")

    # A ledger row must not be able to claim a verdict this tool does not know:
    # a typo'd verdict that silently satisfies the gate is the failure mode a
    # ledger is supposed to prevent.
    merged = load_ledger()
    failures.extend(validate_ledger(merged))

    # And the duplicate-key hook must hand back real dicts, nested objects
    # included -- returning the pair list detects the duplicates and leaves
    # every `rename` / `their_rename` as a list of tuples for everything
    # downstream, which is how a validator ended up rejecting well-formed rows.
    unparsed = sorted(
        k for k, v in merged.items()
        if any(isinstance(x, list) and x and isinstance(x[0], tuple) for x in v.values())
    )
    if unparsed:
        failures.append("load_ledger left nested objects unparsed: %r" % (unparsed[:3],))

    # ...and every upstream-shaped `provides` target must RESOLVE. Two rows
    # named `rclrs::log_trace`, which does not exist -- rclrs stops at `debug`.
    # Generated mechanically from the macro name, which is exactly what the
    # `provides` authoring pass avoided by hand-writing each arrow.
    surfaces = {}
    for lang in ("c", "cpp", "rust"):
        try:
            surfaces[lang] = load_theirs(lang)
        except Exception:  # a surface file may be absent on a partial checkout
            pass
    provides_complaints = _validate_provides_targets(load_ledger(), surfaces)
    failures.extend(provides_complaints)

    # The validator must also CATCH one, or it proves only that today's ledger
    # is today's ledger.
    planted = _validate_provides_targets(
        {"rust:x": {"verdict": "divergence", "why": "x",
                    "provides": ["rclrs::log_trace"]}},
        surfaces,
    )
    if surfaces and not planted:
        failures.append("a `provides` naming a symbol no surface declares was accepted")

    # And the validator itself must reject each shape, or it only proves the
    # ledger on disk is the ledger on disk.
    bad = {
        "cpp:A": {"verdict": "typo", "why": "x", "_shard": "cpp"},
        "cpp:B": {"verdict": "gap", "why": "  ", "_shard": "cpp"},
        "cpp:E": {"verdict": "gap", "why": "x", "_shard": "nonsense",
                  "_file": "nonsense.json"},
        "go:D": {"verdict": "gap", "why": "x", "_shard": "go"},
    }
    caught = validate_ledger(bad)
    for needle in ("unknown verdict", "empty reason", "is not a topic",
                   "unknown language"):
        if not any(needle in c for c in caught):
            failures.append("validate_ledger missed %r" % needle)

    # `their-rename` says the defect is UPSTREAM's, so it is the one verdict a
    # row could claim to excuse itself. Each way of claiming it without the
    # evidence has to be caught, and the well-formed row has to be accepted --
    # a validator that rejects everything is as useless as one that rejects
    # nothing.
    good_tr = {
        "verdict": "their-rename",
        "why": "x",
        "their_rename": {
            "ours": "nros_clock_is_valid",
            "majority": ["rcl: 14 x `rcl_*_is_valid`"],
            "outlier": "rcl_clock_valid",
        },
    }
    if validate_their_rename("c:clock_valid", good_tr):
        failures.append("a well-formed their-rename row was rejected")
    tr_cases = [
        ("needs a `their_rename` object",
         {"verdict": "their-rename", "why": "x"}),
        ("belongs only",
         dict(good_tr, verdict="declined")),
        ("is missing 'outlier'",
         {"verdict": "their-rename", "why": "x",
          "their_rename": {"ours": "a", "majority": ["rclcpp: b"]}}),
        ("cites no upstream",
         {"verdict": "their-rename", "why": "x",
          "their_rename": {"ours": "a", "outlier": "b",
                           "majority": ["our own C and C++ agree"]}}),
        ("must be a list",
         {"verdict": "their-rename", "why": "x",
          "their_rename": {"ours": "a", "outlier": "b", "majority": "rclcpp"}}),
    ]
    for needle, row in tr_cases:
        if not any(needle in c for c in validate_their_rename("c:x", row)):
            failures.append("validate_their_rename missed %r" % needle)

    # A type-level row covers its members only when both sit in the same
    # bucket. The negative case is the one that matters: a gap INSIDE a type we
    # ship is a different claim from not shipping the type.
    led = {"cpp:Node": {"verdict": "gap", "why": "x"}}
    got, inh = lookup(led, "cpp", "Node::create_wall_timer", "theirs-only",
                      {"Node": "theirs-only"})
    if not (got and inh):
        failures.append("a member did not inherit its type's verdict")
    got, inh = lookup(led, "cpp", "Node::create_wall_timer", "theirs-only",
                      {"Node": "same"})
    if got is not None:
        failures.append("a member inherited a verdict across differing buckets")
    got, inh = lookup(led, "cpp", "Node", "theirs-only", {"Node": "theirs-only"})
    if got is None or inh:
        failures.append("a type's own row was reported as inherited")

    # A glob covers a family, but only inside the bucket it declares.
    globbed = {
        "c:action_*": {"verdict": "gap", "why": "x", "bucket": "theirs-only"},
        "c:action_server_init": {"verdict": "divergence", "why": "y"},
    }
    got, inh = lookup(globbed, "c", "action_publish_feedback", "theirs-only", {})
    if not (got and inh and got["verdict"] == "gap"):
        failures.append("a glob row did not cover its family")
    got, _ = lookup(globbed, "c", "action_publish_feedback", "differs", {})
    if got is not None:
        failures.append("a glob row covered a bucket it did not declare")
    got, inh = lookup(globbed, "c", "action_server_init", "theirs-only", {})
    if not got or inh or got["verdict"] != "divergence":
        failures.append("an exact row lost to a glob")
    if not any("must declare the bucket" in c for c in validate_ledger(
            {"c:foo_*": {"verdict": "gap", "why": "x"}})):
        failures.append("a bucketless glob row was accepted")

    # ---------------------------------------------------------------- W0.a
    # issue 1020: the C++ lane must SEE the compat shim, and must not merge
    # what it sees with the native surface.

    # The table itself. A shim TU that quietly reverts to `{"nros"}` roots would
    # parse the header, extract nothing, and report the fix as done.
    compat = [t for t in CPP_TRANSLATION_UNITS if t[0] == "compat"]
    check("the compat shim is a translation unit", len(compat), 1)
    if compat:
        _label, source, extra, roots, marks = compat[0]
        # NOT a filename check any more. phase-417 stage 6 step A moved every
        # `rclcpp::` declaration into the header that owns the concept and
        # DELETED `rclcpp_compat.hpp`; the old assertion named that file and so
        # failed on the step succeeding. What actually matters is that the TU
        # parses a real header of ours -- a shim TU pointed at nothing would
        # extract nothing and report the ported surface as empty, which reads
        # like perfect parity.
        check("the compat TU reads one of our headers",
              source.startswith('#include "nros/') and source.rstrip().endswith('"'),
              True)
        check("the compat TU admits namespace rclcpp", "rclcpp" in roots, True)
        # Decision 3: the shim's rows are std-only, the same marking the `std`
        # TU sets. Not a cosmetic tag -- a `no_std` consumer reaches none of it.
        check("the compat TU marks its rows std_only", marks.get("std_only"), True)
        check("the compat TU marks its rows ported", marks.get("surface"), PORTED)
        check("the compat TU is built under NROS_CPP_STD",
              "-DNROS_CPP_STD=1" in extra, True)
    check("the native TUs stay on the nros namespace",
          [t[3] for t in CPP_TRANSLATION_UNITS if t[0] != "compat"],
          [{"nros"}, {"nros"}, {"nros"}])

    # Decision 1: a `rclcpp::` alias resolving to a `nros::` type correlates
    # `same` on NAME -- and the row says the shape underneath is ours, because
    # `surface` rides along. An alias contributes ONE key and no members, so
    # this cannot make the measurement rosier than it is: the known defect
    # `rclcpp::Publisher<T>::SharedPtr` is still not a row.
    alias_ours = correlate.flatten(
        [{"kind": "alias", "qual": "rclcpp::Publisher", "name": "Publisher",
          "type": "nros::Publisher<M>", "surface": PORTED, "std_only": True}],
        "c++", "ours")
    alias_rows = correlate.compare(
        alias_ours,
        correlate.flatten(
            [{"kind": "type", "qual": "rclcpp::Publisher", "name": "Publisher",
              "members": []}],
            "c++", "theirs"),
        "c++")
    check("a shim alias correlates on name",
          {r["key"]: r["bucket"] for r in alias_rows}.get("Publisher"), "same")
    check("a shim alias says whose shape it is",
          alias_ours["Publisher"]["surface"], PORTED)
    check("a shim alias contributes no members", sorted(alias_ours), ["Publisher"])

    # The marks must SURVIVE `flatten`. They did not before: `std_only` was set
    # on the record and dropped one call later, so it had no consumer anywhere.
    both = correlate.flatten(
        [{"kind": "type", "qual": "nros::Node", "name": "Node",
          "members": [{"name": "spin", "params": [], "ret": "", "template": []}]},
         {"kind": "type", "qual": "rclcpp::Node", "name": "Node",
          "surface": PORTED, "std_only": True,
          "members": [{"name": "spin", "params": [], "ret": "", "template": []},
                      {"name": "pump", "params": [], "ret": "", "template": []}]}],
        "c++", "ours")
    check("a key both surfaces declare is native", both["Node"]["surface"], NATIVE)
    check("a key only the shim declares is ported",
          both["Node::pump"]["surface"], PORTED)
    check("std_only is an AND over the records that merged",
          both["Node::spin"]["std_only"], False)
    check("std_only survives on a shim-only item",
          both["Node::pump"]["std_only"], True)

    # Decision 2: two surfaces, not one. The merge is LOSSY -- an agreeing
    # overload from the shim turns a real `systematic` into `same` -- so the
    # native answer has to be re-correlated, never derived from the merged row.
    theirs_two = correlate.flatten(
        [{"kind": "type", "qual": "rclcpp::Node", "name": "Node",
          "members": [{"name": "create_publisher",
                       "params": [{"type": "const std::string &"}],
                       "ret": "", "template": []}]}],
        "c++", "theirs")
    native_only = correlate.flatten(
        [{"kind": "type", "qual": "nros::Node", "name": "Node",
          "members": [{"name": "create_publisher",
                       "params": [{"type": "const char *"}, {"type": "uint8_t"}],
                       "ret": "", "template": []}]}],
        "c++", "ours")
    merged = correlate.flatten(
        [{"kind": "type", "qual": "nros::Node", "name": "Node",
          "members": [{"name": "create_publisher",
                       "params": [{"type": "const char *"}, {"type": "uint8_t"}],
                       "ret": "", "template": []}]},
         {"kind": "type", "qual": "rclcpp::Node", "name": "Node",
          "surface": PORTED,
          "members": [{"name": "create_publisher",
                       "params": [{"type": "const std::string &"}],
                       "ret": "", "template": []}]}],
        "c++", "ours")
    nb = {r["key"]: r["bucket"]
          for r in correlate.compare(native_only, theirs_two, "c++")}
    pb = {r["key"]: r["bucket"]
          for r in correlate.compare(merged, theirs_two, "c++")}
    check("the shim can mask a native divergence",
          (nb.get("Node::create_publisher"), pb.get("Node::create_publisher")),
          ("differs", "same"))

    # `surface_counts` must report BOTH, and must not count a shim-only row
    # against the native surface it does not exist on.
    ported_c, native_c = surface_counts([
        {"key": "A", "bucket": "same", "native_bucket": "arity-only"},
        {"key": "B", "bucket": "ours-only", "native_bucket": None},
    ])
    check("both surfaces are counted",
          (ported_c, native_c),
          ({"same": 1, "ours-only": 1}, {"arity-only": 1}))

    # And the gate must be selectable per surface, or `--check` silently starts
    # gating rows the ledger was never written against.
    gled = {"cpp:X": {"verdict": "gap", "why": "x"}}
    grows = [{"key": "X", "bucket": "differs", "native_bucket": "differs"},
             {"key": "Y", "bucket": "ours-only", "native_bucket": None},
             {"key": "Z", "bucket": "same", "native_bucket": "theirs-only"}]
    check("the native gate ignores a shim-only row",
          gate_rows(gled, "cpp", grows, "native_bucket"),
          [("cpp", "theirs-only", "Z")])
    check("the ported gate sees it",
          gate_rows(gled, "cpp", grows, "bucket"),
          [("cpp", "ours-only", "Y")])

    # ---------------------------------------------------------------- W0.b
    # RFC-0087's four dispositions.
    check("the four dispositions are RFC-0087's",
          set(DISPOSITIONS),
          {"adopt", "adopt-bounded", "refuse-loud", "absent"})
    for good in DISPOSITIONS:
        if validate_ledger({"cpp:A": {"verdict": "declined", "why": "x",
                                      "disposition": good}}):
            failures.append("validate_ledger rejected disposition %r" % good)
    if not any("unknown disposition" in c for c in validate_ledger(
            {"cpp:A": {"verdict": "declined", "why": "x",
                       "disposition": "adopt-ish"}})):
        failures.append("a typo'd disposition was accepted")
    # OPTIONAL, and that is load-bearing until phase-417 W-M2: a required field
    # would fail every `declined` row in the tree the day it landed, and would
    # break whoever is mid-edit on the correction track.
    if validate_ledger({"cpp:A": {"verdict": "declined", "why": "x"}}):
        failures.append("a row without a disposition was rejected; W-M2 turns "
                        "that on, not W0.b")
    check("undisposed finds a declined row with no disposition",
          undisposed({"cpp:A": {"verdict": "declined", "why": "x"},
                      "cpp:B": {"verdict": "declined", "why": "x",
                                "disposition": "refuse-loud"},
                      "cpp:C": {"verdict": "gap", "why": "x"}}),
          ["cpp:A"])

    failures.extend(public_surface.self_test())
    failures.extend(signature_rules.self_test())
    failures.extend(topics.self_test())

    for f in failures:
        print("FAIL " + f, file=sys.stderr)
    print("self-test: %d checks failed" % len(failures))
    return 1 if failures else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--lang", action="append", choices=LANGS, help="default: all three")
    ap.add_argument("--show", action="append", default=None,
                    help="buckets: same, arity-only, systematic, differs, ours-only, theirs-only, all")
    ap.add_argument("--check", action="store_true", help="exit non-zero on an unledgered difference")
    ap.add_argument("--check-ported", action="store_true",
                    help="also gate the ported-file surface (rclcpp_compat.hpp); "
                         "phase-417 W-M2 makes this the default")
    ap.add_argument("--require-disposition", action="store_true",
                    help="with --check, fail when a `declined` row carries no "
                         "RFC-0087 disposition; phase-417 W-M2 makes this the default")
    ap.add_argument("--suggest-renames", action="store_true",
                    help="pair unmatched names by similarity (suggestions, never findings)")
    ap.add_argument("--include-internal", action="store_true",
                    help="do not filter the ROS 2 side down to public API")
    ap.add_argument("--grep", metavar="REGEX",
                    help="list only rows whose key matches; the summary counts stay whole-lane")
    ap.add_argument("--topic", choices=topics.NAMES,
                    help="one stage's rows, in every language")
    ap.add_argument("--by-topic", action="store_true",
                    help="how many decisions each stage still needs, per language")
    ap.add_argument("--refresh", action="store_true", help="re-derive the ROS 2 side and record it")
    ap.add_argument("--ros-prefix", default=os.environ.get("ROS_PREFIX", "/opt/ros/humble"))
    ap.add_argument("--rclc", help="path to a ros2/rclc checkout (for --refresh --lang c)")
    ap.add_argument("--rclrs", help="path to the rclrs crate (for --refresh --lang rust)")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    langs = args.lang or list(LANGS)
    if args.refresh:
        refresh(langs, args.ros_prefix, args.rclc, args.rclrs)
        return 0

    show = set(args.show or ["differs", "arity-only", "systematic",
                             "ours-only", "theirs-only"])
    if args.by_topic:
        with tempfile.TemporaryDirectory() as tmpdir:
            return by_topic(langs, tmpdir)

    grep = re.compile(args.grep) if args.grep else None
    return report(langs, show, args.check, args.suggest_renames,
                  args.include_internal, grep, args.topic,
                  args.check_ported, args.require_disposition)


if __name__ == "__main__":
    sys.exit(main())
