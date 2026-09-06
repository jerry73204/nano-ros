#!/usr/bin/env python3
"""Does an image's `NROS_EXECUTOR_ACTION_CLIENTS` match what it actually links?

Issue 0900, option 3.

WHAT THIS ANSWERS AND WHY IT IS THE ONLY MECHANISM THAT REACHES EMBEDDED

`NROS_EXECUTOR_ACTION_CLIENTS` decides how many of the `MAX_CBS` executor arena
slots are budgeted at ActionClient size rather than pub/sub size. It defaults to
`MAX_CBS`, which reproduces the pre-0900 arithmetic byte for byte: 74,240 bytes
at the shipped defaults against 16,384 with the knob at 0. That arena is INLINE
ON THE TASK STACK (`Executor::open_sized`), not in `.bss`, so `just mem-report`
cannot see it at all and never will — do not try to measure the arena here. What
is checkable is the ENTITY whose size justifies the budget.

An image that never links the action-client creation path cannot have an action
client. That is a post-link fact, so it cannot SIZE a compile-time constant, but
it can GATE one:

  knob > 0, machinery NOT linked -> the image carries arena it cannot use.
      ADVISORY. This is the common case today (the knob defaults to the worst
      case and nothing ever set it), and a gate that fails on every image in the
      tree has no signal capacity — a regression landing in it would look
      exactly like yesterday's red.
  knob == 0, machinery IS linked  -> registration will fail at RUNTIME with
      `NodeError::BufferTooSmall`, on a target where a return code is all you
      get. FAILS. This is the direction that has to be loud.

The sidecar/metadata half of issue 0900 cannot cover embedded at all: a
cross-compiled component is unprobeable by construction (`metadata_build.rs`'s
own test is named `build_std_with_a_foreign_target_is_unprobeable`). `nm` reads a
cross ELF perfectly well, which is why this half exists and why it is the only
one that reaches the targets where a 56.5 KiB stack frame decides whether an
image boots.

WHICH SYMBOL — MEASURED, NOT ASSUMED

The obvious candidate, `ActionClientCore::new`, is WRONG TWICE and a probe on it
would have passed vacuously:

  * `ActionClientCore::new` is inlined away. It appears in NO image in the tree,
    action clients included.
  * `ActionClientCore` as a type appears in EVERY image that links the executor:
    6 symbols (drop glue + five methods) in `qemu-rtic-talker`, a pub/sub-only
    image, identical to `qemu-rtic-action-client`. `spin.rs` names the
    default-sized monomorphisation directly, so the dispatch code is linked
    whether or not anything creates a client.

The discriminator is the CREATION path, `create_action_client*` /
`create_action_server*` on `NodeHandle` / `DeclaredNode`. Measured over 58 built
ELFs on 2026-09-01 — 20 Cortex-M (thumbv7m-none-eabi) and 38 x86_64 — the two
families matched exactly the four action clients and the three action servers,
and nothing else. `service-client` and `service-server` are the nearest misses
and both score 0:

    image                      client  server  ActionClientCore
    qemu-rtic-action-client       1       0          6
    qemu-rtic-action-server       0       1          6
    qemu-rtic-talker              0       0          6
    qemu-rtic-service-client      0       0          6
    action-client (x86_64)        1       0          6
    action-server (x86_64)        0       1          6
    native-rs-async-action-client 1       0          3
    rtic-action-client            1       0          2
    rtic-action-server            0       1          0

The substring survives with or without a demangler: the legacy mangling embeds
`24create_action_client_raw` and v0 embeds the same identifier length-prefixed,
so a host `nm` that cannot demangle Rust degrades to the same verdict rather
than to silence.

WHY AN ACTION *SERVER* COUNTS TOO, THOUGH THE KNOB IS NAMED FOR CLIENTS

The knob's real meaning is "how many slots are budgeted at the WORST-CASE entry
size", and `build.rs` picked the action client as that worst case. It is not:
measured from `target/nros-cpp-generated/nros/nros_cpp_config_generated.h`
(x86_64, default 1024-byte buffers),

    NROS_CPP_RAW_ACTION_CLIENT_OPAQUE_U64S = 654  ->  5,232 B
    NROS_CPP_RAW_ACTION_SERVER_OPAQUE_U64S = 799  ->  6,392 B
# **Caveat on those two numbers, added on integration.** `NROS_CPP_RAW_ACTION_*_
# OPAQUE_U64S` are the C++ FFI OPAQUE HANDLE sizes, not arena entry sizes. The
# arena budgets an action client at 18,048 B from a different formula
# (`3 x (4096+384) + 3 x rx_buf + 1536`), so 6,392 vs 5,232 does NOT say the
# action-server ARENA ENTRY exceeds the client's — only that the C++ handle does.
#
# The conclusion the gate acts on survives the correction, for a different reason:
# the arena demonstrably stores action servers (`ActionServerArenaEntry`,
# `ActionServerRawArenaEntry` in `arena.rs`), so an action-server image occupies
# slots and must not be advised to zero the knob. Treating both entities as heavy
# is conservative and correct whichever handle is larger. The size ORDERING is
# recorded as suggestive, not as the justification.

against a pub/sub entry budgeted at `3 * rx_buf + 512` = 3,584 B. So an action
SERVER also needs a slot a pub/sub budget cannot hold, and telling an
action-server image to set `NROS_EXECUTOR_ACTION_CLIENTS=0` would trade this
gate's advisory for the exact runtime `BufferTooSmall` it exists to prevent. An
image linking either entity is therefore "heavy" here.

WHAT IT CANNOT DECIDE, AND SAYS SO

A C or C++ image links `nros-c` / `nros-cpp` with `--whole-archive` (the issue
0475 shape). The whole staticlib is therefore in the image whether or not the
app calls any of it, and `NodeHandle::create_action_client_raw` is present as a
GLOBAL in every C entry — `native_entry` (a talker) and `native_action_client_entry`
are byte-for-byte indistinguishable on this probe. Such an image is reported
INDETERMINATE, never OK: presence proves nothing there. It is detected by the
FFI surface itself (`nros_cpp_action_client_create` / `nros_action_client_init`),
which is absent from every pure-Rust image measured.

Images built by CMake outside a cargo profile directory (the `examples/
workspaces/*/build/*/cmake/*` entries) are out of reach for a second, independent
reason: there is no `nros_node_config.rs` this tool can attribute to them, so the
knob half of the comparison is missing too. Both halves fail for the same
population, which is why not covering it costs nothing that was ever available.

WHERE IT LOOKS, AND WHY IT IS NOT THE REPOSITORY (phase-413 W7, issue 1001)

The roots are DERIVED from `fixtures-manifest.py` — the same join
`nros_fixture_row_artifact_dir` makes in shell (issue 1025) — plus the west
lane's build dirs and the root workspace's own `target/`. It used to walk the
whole checkout, which is 240,754 directories post-`PRUNE` on a developer tree:
0.6 s warm, past 600 s cold, on the one lane the `pre-push` hook runs alone.

Every way of scoping the WALK was measured and rejected. Pruning inside the
anchor's own `build/<pkg>-<hash>/out` shape saves 0.0 % once the predicate is
correct — attempt 2's entire reported win WAS the amputation that silently
dropped 159 of 410 findings — and splitting on `CACHEDIR.TAG` caps the saving at
47 % while losing 223 of 2,415 anchors, because cargo driven by Corrosion, CMake
and west writes no tag. `--all-roots` restores the whole-checkout walk for an
audit; it is the switch that re-measures what the derived set does not reach.

AND IT FAILS CLOSED

Examining nothing is a FAILURE, not a skip. The gate runs where a fixture build
has just finished, so "no image with an attributable config" means the build did
not happen. It was fail-open for as long as it sat on the fast line — it skipped
in most CI runs, examined nothing on a pull request, and charged the one person
who had images — which is what `check-gate-visibility` was objecting to when it
refused the build tier.

Usage::

    check-action-client-arena-budget.py                # the gate
    check-action-client-arena-budget.py --root DIR ... # extra trees to examine
    check-action-client-arena-budget.py --all-roots    # walk the whole checkout
    check-action-client-arena-budget.py -v             # every image, not just findings

Exit codes: 0 = agreed (or only advisories), 1 = an image will fail at
registration, OR nothing was examined.
"""

import argparse
import importlib.util
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The creation paths. Deliberately the whole `create_action_client` /
# `create_action_server` families (`_raw`, `_sized`, `_with_callbacks`,
# `_with_callbacks_for_name`, …) rather than one spelling: which arm survives
# monomorphisation depends on how the app reached it, and they are all the same
# fact.
CREATE_CLIENT = re.compile(r"create_action_client")
CREATE_SERVER = re.compile(r"create_action_server")

# The C/C++ FFI surface. Its PRESENCE means the probe above cannot decide,
# because these staticlibs are whole-archived.
FFI_SURFACE = re.compile(r"\bnros_(?:cpp_)?action_client_(?:create|init)\b")

# `pub const NAME: usize = 123;` out of the generated `nros_node_config.rs`.
CONST = re.compile(r"^\s*pub const ([A-Z0-9_]+):\s*usize\s*=\s*([0-9_]+)\s*;", re.M)

# Directories that can never hold a final image but cost the most to walk.
# `deps`/`incremental`/`.fingerprint` are cargo's intermediates; the rest are
# the tree's known-heavy corners.
PRUNE = {
    ".git",
    ".jj",
    ".fingerprint",
    "incremental",
    "deps",
    "node_modules",
    ".venv",
    "__pycache__",
    "third-party",
}

# Extensions that are never an IMAGE. Shared objects are excluded on purpose:
# the arena lives on the stack of whatever task calls spin, and a library has no
# task.
NOT_AN_IMAGE = {
    ".d",
    ".rlib",
    ".rmeta",
    ".o",
    ".a",
    ".so",
    ".dylib",
    ".json",
    ".toml",
    ".txt",
    ".map",
    ".lock",
    ".timestamp",
}

CONFIG_NAME = "nros_node_config.rs"


# --------------------------------------------------------------------------
# tool resolution — reuse `nros-mem-report.py`'s, do not grow a second spelling
# --------------------------------------------------------------------------


def _mem_report():
    """Import `scripts/nros-mem-report.py` for its nm resolution.

    Hyphenated filename, so importlib rather than `import`. That script already
    knows the two things worth knowing about `nm` here — GNU nm refuses a
    foreign ELF, and rustup ships an llvm-nm that reads them all but is not on
    PATH — and duplicating either is how a second, worse copy of `pick_nm` gets
    written. Same importlib move `nros-mem-report.py` itself makes for
    `gen-pool-inventory.py`.
    """
    path = os.path.join(ROOT, "scripts", "nros-mem-report.py")
    spec = importlib.util.spec_from_file_location("nros_mem_report", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_MEM = None


def nm_for(elf):
    """An `nm` that can read this ELF, or None."""
    global _MEM
    if _MEM is None:
        _MEM = _mem_report()
    try:
        return _MEM.pick_nm(elf)
    except SystemExit:
        # `pick_nm` exits when no nm can read the file. Here that is a SKIP for
        # one image, not the end of the gate.
        return None


# --------------------------------------------------------------------------
# the pure classifier — this is what the selftest exercises
# --------------------------------------------------------------------------

MATCHED = "matched"
OVER = "over-budgeted"
UNDER = "under-budgeted"
INDETERMINATE = "indeterminate"


def classify(action_clients, max_cbs, nm_text):
    """(verdict, detail) for one image.

    `nm_text` is the demangled symbol dump. `action_clients` is the image's
    `ARENA_ACTION_CLIENTS`; `max_cbs` its `MAX_CBS`, used only to say whether
    the knob was CHOSEN or merely defaulted.
    """
    if not nm_text.strip():
        return INDETERMINATE, "no symbol table (stripped?) — nothing to read"
    if FFI_SURFACE.search(nm_text):
        return (
            INDETERMINATE,
            "links the nros-c/nros-cpp FFI, which is whole-archived: the "
            "action-client machinery is present whether or not the app uses it",
        )

    found = []
    if CREATE_CLIENT.search(nm_text):
        found.append("action client")
    if CREATE_SERVER.search(nm_text):
        found.append("action server")

    if action_clients == 0 and found:
        return (
            UNDER,
            f"NROS_EXECUTOR_ACTION_CLIENTS=0 but the image LINKS "
            f"{' and '.join(found)} creation — registering it will fail at "
            f"RUNTIME with NodeError::BufferTooSmall, not at link",
        )
    if action_clients > 0 and not found:
        chosen = "explicitly set" if action_clients != max_cbs else "defaulted"
        return (
            OVER,
            f"{action_clients} of {max_cbs} arena slots budgeted at "
            f"worst-case (ActionClient) size ({chosen}) but the image links "
            f"neither an action client nor an action server — that arena is "
            f"inline on the task stack; set NROS_EXECUTOR_ACTION_CLIENTS=0",
        )
    if action_clients == 0:
        return MATCHED, "no action entity linked, none budgeted"
    return MATCHED, f"{action_clients} slot(s) budgeted, {' and '.join(found)} linked"


def parse_config(text):
    """The `usize` consts of a generated `nros_node_config.rs`."""
    return {m.group(1): int(m.group(2).replace("_", "")) for m in CONST.finditer(text)}


# --------------------------------------------------------------------------
# discovery
# --------------------------------------------------------------------------


def is_elf_image(path):
    """True for an ET_EXEC/ET_DYN ELF. Cheap: 18 bytes, no subprocess."""
    if os.path.splitext(path)[1] in NOT_AN_IMAGE:
        return False
    try:
        if os.path.islink(path) or not os.path.isfile(path):
            return False
        with open(path, "rb") as fh:
            head = fh.read(18)
    except OSError:
        return False
    if len(head) < 18 or head[:4] != b"\x7fELF":
        return False
    little = head[5] == 1
    e_type = int.from_bytes(head[16:18], "little" if little else "big")
    return e_type in (2, 3)  # ET_EXEC, ET_DYN


# --------------------------------------------------------------------------
# where to look — the manifest, not the repository (phase-413 W7, issue 1001)
# --------------------------------------------------------------------------

MANIFEST = os.path.join(ROOT, "scripts", "build", "fixtures-manifest.py")


def _manifest(subcommand):
    """Rows of `fixtures-manifest.py <subcommand>`, split on the US separator.

    A failure here is FATAL rather than an empty root set: an empty root set
    looks exactly like "nothing is built", and issue 1001's whole lesson is
    that this tool reports the same SHAPE while examining a smaller set.
    """
    out = subprocess.run(
        [sys.executable, MANIFEST, subcommand],
        capture_output=True, text=True, check=True,
    ).stdout
    return [ln.split("\x1f") for ln in out.splitlines() if ln.strip()]


def derived_roots():
    """The directories a fixture build actually writes images into.

    Issue 1001. This used to be `[ROOT]` — the whole checkout — which is
    240,754 directories post-`PRUNE` on a developer tree, 0.6 s warm and past
    600 s cold, on the lane the `pre-push` hook runs alone.

    Every candidate for scoping the WALK was measured and rejected (phase-413
    W7): pruning inside the anchor's own `build/<pkg>-<hash>/out` shape saves
    0.0 % once its predicate is correct — attempt 2's entire reported win was
    the amputation that dropped 159 of 410 findings — and splitting on
    `CACHEDIR.TAG` caps the saving at 47 % while losing 223 of 2,415 anchors,
    because cargo driven by Corrosion, CMake and west writes no tag.

    So the root set is DERIVED instead, from the manifest that already decides
    where each row builds. `nros_fixture_row_artifact_dir` is the shell
    spelling of the same join (issue 1025); `fixture-groups` is that function's
    two inputs already resolved, so this is not a second derivation of the
    group key — it is the same one, read.
    """
    build_root = os.environ.get("NROS_BUILD_ROOT") or os.path.join(ROOT, "build")
    zephyr_root = (os.environ.get("NROS_ZEPHYR_BUILD_ROOT")
                   or os.path.join(ROOT, "zephyr-workspace"))

    roots = set()
    # `<artifact_root>\x1f<platform>\x1f<group_slug>\x1f<eligible>` — an empty
    # slug means the platform does not share, so the row keeps its leaf `target`.
    for row in _manifest("fixture-groups"):
        artifact_root, slug = row[0], (row[2] if len(row) > 2 else "")
        roots.add(os.path.join(build_root, "cargo-fixtures", slug) if slug
                  else os.path.join(ROOT, artifact_root))
    # Workspace rows carry their own `--target-dir` NAME (`target-fixtures`)
    # rather than a group slug: field 2 is the workspace, field 6 the dir.
    for row in _manifest("list-workspaces"):
        if len(row) > 6 and row[2] and row[6]:
            roots.add(os.path.join(ROOT, row[2], row[6]))
    # The west lane builds outside cargo's layout; field 6 is its build-dir name.
    for row in _manifest("west-leaves"):
        if len(row) > 6 and row[6]:
            roots.add(os.path.join(zephyr_root, row[6]))
    # Two named dirs under `<build_root>` that no manifest row places: the
    # metadata half of issue 0900 builds host probes in the first, and CMake
    # reaches cargo through Corrosion into the second — which is where the
    # NuttX cross images land, exactly the population this gate exists for.
    roots.add(os.path.join(build_root, "metadata-probe"))
    roots.add(os.path.join(build_root, "corrosion-cargo"))
    # The root workspace's own target dir. One named directory, not a walk: the
    # `nros-tests` bins and the reference entries build here with no manifest row.
    roots.add(os.path.join(ROOT, "target"))
    return sorted(roots)


def find_image_roots(roots):
    """{cargo profile dir: [generated config path, …]}.

    The anchor is the generated config, not the binary: a profile dir is the one
    place where the knob a binary compiled against and the binary itself are
    both on disk, so attribution is a directory fact rather than a guess.
    """
    found = {}
    for root in roots:
        if not os.path.isdir(root):
            continue
        # walk-ok: `roots` are BUILD directories, and what this looks for —
        # a compiled binary beside the knob it was compiled against — is
        # untracked by construction, so `git ls-files` cannot see it. That is
        # the case the gate's own text carves out ("scanning for UNTRACKED
        # artifacts is fine — scope it to a build dir"), and PRUNE keeps it
        # off the vendored trees.
        for dirpath, dirnames, filenames in os.walk(root):
            base = os.path.basename(dirpath)
            if base in PRUNE:
                dirnames[:] = []
                continue
            dirnames[:] = [d for d in dirnames if d not in PRUNE]
            # `<profile>/build/<pkg>-<hash>/out/` — do not descend past `out`,
            # which for some crates is a whole vendored build tree.
            if base == "out" and os.path.basename(os.path.dirname(os.path.dirname(dirpath))) == "build":
                dirnames[:] = []
                if CONFIG_NAME in filenames:
                    profile = os.path.dirname(os.path.dirname(os.path.dirname(dirpath)))
                    found.setdefault(profile, []).append(os.path.join(dirpath, CONFIG_NAME))
    return found


def images_in(profile_dir):
    """Final artifacts of a cargo profile dir: its own files plus `examples/`."""
    out = []
    for sub in ("", "examples"):
        d = os.path.join(profile_dir, sub) if sub else profile_dir
        try:
            names = sorted(os.listdir(d))
        except OSError:
            continue
        for n in names:
            p = os.path.join(d, n)
            if is_elf_image(p):
                out.append(p)
    return out


def knob_of(config_paths):
    """(action_clients, max_cbs, arena_size) or (None, reason).

    A profile dir accumulates one `build/<pkg>-<hash>/out` per feature set AND
    keeps stale ones from earlier builds, so this has to be explicit about
    disagreement rather than picking the first. Configs written before issue
    0900 W2 have no `ARENA_ACTION_CLIENTS` at all and cannot answer; they are
    ignored, and if none remain the whole root is skipped.
    """
    seen = set()
    arenas = set()
    ignored = 0
    for p in config_paths:
        try:
            with open(p, encoding="utf8") as fh:
                consts = parse_config(fh.read())
        except OSError:
            continue
        if "ARENA_ACTION_CLIENTS" not in consts:
            ignored += 1
            continue
        seen.add((consts["ARENA_ACTION_CLIENTS"], consts.get("MAX_CBS", 0)))
        arenas.add(consts.get("ARENA_SIZE", 0))
    if not seen:
        return None, (
            f"no generated config declares ARENA_ACTION_CLIENTS "
            f"({ignored} predate issue 0900 W2)"
        )
    if len(seen) > 1:
        vals = ", ".join(str(k[0]) for k in sorted(seen))
        return None, f"generated configs disagree on ARENA_ACTION_CLIENTS ({vals})"
    action_clients, max_cbs = next(iter(seen))
    # ARENA_SIZE may legitimately VARY inside one profile dir where a leaf sets
    # `NROS_EXECUTOR_ARENA_SIZE` explicitly (the esp32 fixtures do: 16,384
    # beside 74,240) while the derivation input is the same. That is not a
    # disagreement about the KNOB, so it does not block the verdict — it is
    # reported as a range.
    arena = "/".join(str(a) for a in sorted(arenas))
    return (action_clients, max_cbs, arena), None


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------


def display(path):
    """Repo-relative when the path is inside the repo, absolute otherwise.

    A blind `relpath` against ROOT turns an `--root` outside the checkout into a
    ladder of `../..` that nobody can read or paste back.
    """
    rel = os.path.relpath(path, ROOT)
    return path if rel.startswith("..") else rel


def symbols(elf):
    """Demangled `nm` output, or None if no tool can read this file."""
    tool = nm_for(elf)
    if not tool:
        return None
    run = subprocess.run([tool, "-C", elf], capture_output=True, text=True)
    if run.returncode != 0:
        return None
    return run.stdout


def main(argv=None, _self_test=True):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument(
        "--root",
        action="append",
        default=[],
        help="extra tree to examine (repeatable). Default: the repo and "
        "$NROS_BUILD_ROOT.",
    )
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print every image, not only the findings")
    ap.add_argument(
        "--all-roots",
        action="store_true",
        help="walk the whole checkout instead of the manifest's roots. An "
        "AUDIT switch: it is minutes on a cold page cache (issue 1001) and is "
        "how you re-measure what the derived root set does not reach.",
    )
    args = ap.parse_args(argv)

    # `_self_test=False` only from inside `self_test` itself, which drives this
    # function to prove the empty-root verdict; re-entering would not terminate.
    if _self_test:
        self_test()

    roots = [ROOT] if args.all_roots else derived_roots()
    roots.extend(args.root)

    verdicts = {MATCHED: [], OVER: [], UNDER: [], INDETERMINATE: []}
    skipped_roots = []

    for profile, configs in sorted(find_image_roots(roots).items()):
        imgs = images_in(profile)
        if not imgs:
            continue
        knob, why = knob_of(configs)
        if knob is None:
            skipped_roots.append((profile, why, len(imgs)))
            continue
        action_clients, max_cbs, arena = knob
        for elf in imgs:
            text = symbols(elf)
            if text is None:
                verdicts[INDETERMINATE].append(
                    (elf, arena, "no `nm` on this host can read this ELF")
                )
                continue
            verdict, detail = classify(action_clients, max_cbs, text)
            verdicts[verdict].append((elf, arena, detail))

    examined = sum(len(v) for v in verdicts.values())
    if examined == 0:
        # FAILURE, not a skip. This gate runs where a fixture build has just
        # finished, so "no image with an attributable config" means the build
        # did not happen — and a check that verified nothing must never read as
        # a pass (issue 1001; the same lesson as `check-no-vacuous-tests` and
        # `check-lane-contracts`). It was fail-open for as long as it sat on the
        # fast line, which is what `check-gate-visibility` was objecting to.
        print("check-action-client-arena-budget: examined NOTHING.", file=sys.stderr)
        print("  - no built image with an attributable nros_node_config.rs was "
              "found under:", file=sys.stderr)
        for r in roots:
            print(f"      {display(r)}", file=sys.stderr)
        for profile, why, n in skipped_roots:
            print(f"  - {display(profile)}: {why} ({n} image(s))", file=sys.stderr)
        print("\n  Build first (`just build-test-fixtures lane=native`), or pass "
              "--root DIR\n  for a tree outside the manifest. `--all-roots` walks "
              "the whole checkout.", file=sys.stderr)
        return 1

    def show(name, rows, prefix):
        for elf, arena, detail in rows:
            print(f"  {prefix} {display(elf)}\n      ARENA_SIZE={arena} B — {detail}")

    if verdicts[UNDER]:
        print(
            f"check-action-client-arena-budget: {len(verdicts[UNDER])} image(s) "
            f"will fail at REGISTRATION:",
            file=sys.stderr,
        )
        show(UNDER, verdicts[UNDER], "FAIL")
        print(
            "\n  The arena budgets 0 slots at ActionClient size and the image "
            "links one.\n"
            "  Raise NROS_EXECUTOR_ACTION_CLIENTS (or NROS_EXECUTOR_ARENA_SIZE) "
            "for these\n  images, or stop linking the action client. Issue 0900."
        )
        return 1

    if verdicts[OVER]:
        print(
            f"check-action-client-arena-budget: {len(verdicts[OVER])} image(s) "
            f"carry arena they cannot use (ADVISORY, issue 0900):"
        )
        show(OVER, verdicts[OVER], "over ")
        print(
            "  Not a failure: the knob defaults to the worst case, so this is "
            "the shipped\n  state of most images and a red here would carry no "
            "signal. The saving is\n  stack, not .bss — `just mem-report` cannot "
            "see it."
        )

    if args.verbose:
        show(MATCHED, verdicts[MATCHED], "ok   ")
        show(INDETERMINATE, verdicts[INDETERMINATE], "?    ")

    for profile, why, n in skipped_roots:
        print(f"  [not examined] {display(profile)}: {why} ({n} image(s))")

    print(
        f"check-action-client-arena-budget OK — {examined} image(s): "
        f"{len(verdicts[MATCHED])} agreed, {len(verdicts[OVER])} over-budgeted, "
        f"{len(verdicts[INDETERMINATE])} undecidable "
        f"(C/C++ FFI whole-archive or no symtab)."
    )
    return 0


# --------------------------------------------------------------------------
# selftest — runs on the NORMAL path, never behind a flag (phase-395)
# --------------------------------------------------------------------------

# Symbol dumps in the exact shape `nm -C` produces, lifted from real images so
# the cases are the measurements rather than a paraphrase of them.
_TALKER = """\
0002dd11 t nros_node::executor::arena::action_client_raw_try_process::<1024, 1024, 1024>
0002d24b t core::ptr::drop_glue::<nros_node::executor::action_core::ActionClientCore<1024, 1024, 1024>>
0002b037 t <nros_node::executor::action_core::ActionClientCore<1024, 1024, 1024>>::send_goal_raw
00020b00 T nros_board_native_run_components
"""

_ACTION_CLIENT = _TALKER + """\
00001433 t <nros::node::DeclaredNode>::create_action_client_with_callbacks_for_name::<example_interfaces::action::fibonacci::Fibonacci>
"""

_ACTION_SERVER = _TALKER + """\
0002ab11 t <nros::node::DeclaredNode>::create_action_server_with_callbacks_for_name::<example_interfaces::action::fibonacci::Fibonacci>
"""

# The C entry: the FFI surface is whole-archived, so `create_action_client_raw`
# is present in a TALKER. This is the case that makes presence undecidable.
_C_ENTRY = _TALKER + """\
0010b630 T <nros_node::executor::node::NodeHandle>::create_action_client_raw
000e2ee0 T nros_cpp_action_client_create
"""


def self_test(quiet=True):
    """Prove both verdicts, every run.

    A negative control nobody runs decays into a comment — and this gate's whole
    reason for existing is that the OBVIOUS probe (`ActionClientCore`) passes
    vacuously, so "it printed OK" is worth nothing here without a demonstrated
    red. `_TALKER` therefore carries the six `ActionClientCore` symbols that a
    naive probe would have matched: case 5 fails the moment someone widens
    `CREATE` to the type name.
    """
    cases = [
        # (action_clients, max_cbs, nm text, expected verdict)
        # The dangerous direction: budgeted for none, links one.
        (0, 4, _ACTION_CLIENT, UNDER),
        # The common direction: budgeted for four, links none.
        (4, 4, _TALKER, OVER),
        # Agreement, both ways round.
        (4, 4, _ACTION_CLIENT, MATCHED),
        (0, 4, _TALKER, MATCHED),
        # An action SERVER holds a slot a pub/sub budget cannot fit either
        # (6,392 B measured against 3,584 B), so it counts as heavy. Without
        # these two cases the gate would advise an action-server image to zero
        # the knob and cause the very failure it is meant to catch.
        (0, 4, _ACTION_SERVER, UNDER),
        (4, 4, _ACTION_SERVER, MATCHED),
        # A C/C++ image can never be decided on symbols.
        (0, 4, _C_ENTRY, INDETERMINATE),
        (4, 4, _C_ENTRY, INDETERMINATE),
        # A stripped image is a skip, not a pass.
        (0, 4, "", INDETERMINATE),
        # An explicitly-chosen-but-still-unused budget is still only advisory.
        (1, 4, _TALKER, OVER),
    ]
    fails = []
    for i, (ac, cbs, text, want) in enumerate(cases):
        got = classify(ac, cbs, text)[0]
        if got != want:
            fails.append(f"classify case {i}: expected {want}, got {got}")

    # The knob side has its own failure mode: a profile dir with a stale config
    # beside a current one must not silently pick one.
    parsed = parse_config(
        "pub const MAX_CBS: usize = 4;\n"
        "pub const ARENA_SIZE: usize = 74_240;\n"
        "pub const ARENA_ACTION_CLIENTS: usize = 4;\n"
    )
    if parsed != {"MAX_CBS": 4, "ARENA_SIZE": 74240, "ARENA_ACTION_CLIENTS": 4}:
        fails.append(f"parse_config: got {parsed}")

    import tempfile

    with tempfile.TemporaryDirectory() as d:
        cur = os.path.join(d, "cur.rs")
        old = os.path.join(d, "old.rs")
        other = os.path.join(d, "other.rs")
        with open(cur, "w", encoding="utf8") as fh:
            fh.write("pub const MAX_CBS: usize = 4;\npub const ARENA_ACTION_CLIENTS: usize = 4;\n")
        with open(old, "w", encoding="utf8") as fh:
            fh.write("pub const MAX_CBS: usize = 4;\n")  # predates the knob
        with open(other, "w", encoding="utf8") as fh:
            fh.write("pub const MAX_CBS: usize = 4;\npub const ARENA_ACTION_CLIENTS: usize = 0;\n")
        if knob_of([cur, old])[0] != (4, 4, "0"):
            fails.append("knob_of: a pre-0900 config must be ignored, not fatal")
        if knob_of([old])[0] is not None:
            fails.append("knob_of: only pre-0900 configs must yield no answer")
        if knob_of([cur, other])[0] is not None:
            fails.append("knob_of: disagreeing configs must yield no answer")

        # `is_elf_image` gates every walk result; a wrong answer here silently
        # shrinks the population the gate examines.
        elf = os.path.join(d, "exe")
        with open(elf, "wb") as fh:
            fh.write(b"\x7fELF\x02\x01\x01" + b"\0" * 9 + (2).to_bytes(2, "little"))
        obj = os.path.join(d, "obj")
        with open(obj, "wb") as fh:
            fh.write(b"\x7fELF\x02\x01\x01" + b"\0" * 9 + (1).to_bytes(2, "little"))
        text_file = os.path.join(d, "notelf")
        with open(text_file, "w", encoding="utf8") as fh:
            fh.write("hello")
        if not is_elf_image(elf):
            fails.append("is_elf_image: ET_EXEC must count")
        if is_elf_image(obj):
            fails.append("is_elf_image: ET_REL (an object file) must not count")
        if is_elf_image(text_file):
            fails.append("is_elf_image: a non-ELF must not count")

    # Phase-413 W7 / issue 1001. Two properties of the scoping, both of which
    # a "it printed OK" reading cannot distinguish from their absence.
    #
    # 1. The derived root set is NON-EMPTY and holds no duplicates. An empty
    #    root set examines nothing while printing the same shape — which is
    #    exactly how attempt 2 passed inspection while dropping 159 findings.
    roots = derived_roots()
    if not roots:
        fails.append("derived_roots: empty — the manifest yielded no root at all")
    if len(roots) != len(set(roots)):
        fails.append("derived_roots: duplicate roots")
    if not any(os.path.basename(r) == "target" for r in roots):
        fails.append("derived_roots: the root workspace's own target/ is missing")
    # 2. Examining nothing is a FAILURE. This gate was fail-open for as long as
    #    it sat on the fast line, and that is the defect the move fixes; a
    #    regression here would restore a green over zero images.
    _saved = globals()["derived_roots"]
    globals()["derived_roots"] = lambda: []
    try:
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            rc_empty = main([], _self_test=False)
    finally:
        globals()["derived_roots"] = _saved
    if rc_empty != 1:
        fails.append(f"examining nothing must exit 1, got {rc_empty}")

    if fails:
        for f in fails:
            print(f"check-action-client-arena-budget self-test: FAIL {f}", file=sys.stderr)
        raise SystemExit(1)
    if not quiet:
        print("check-action-client-arena-budget self-test: OK")
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv or "--selftest" in sys.argv:
        sys.exit(self_test(quiet=False))
    sys.exit(main())
