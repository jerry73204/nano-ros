#!/usr/bin/env python3
"""Issue 1028 — an RTOS that names itself is not a host.

`"Is this target hosted?"` and `"does `target_os` have a value?"` are different
questions. NuttX is the counterexample that proves it: `armv7a-nuttx-eabihf`
reports `target_os = "nuttx"` and `target_family = "unix"`, so the obvious test
`target_os != "none"` reads an RTOS on a fixed-RAM part as a Linux-class host.

Measured cost the one time it was looked at:
`examples/qemu-arm-nuttx/cpp/action-client`, an image that opens ZERO
queryables, carried a 32-slot `SERVICE_BUFFERS` table — 142,336 B of `.bss`
where the embedded budget gives 35,584 B. 106,752 B, byte-identical in the C
image, decided by a predicate that could not tell which kind of machine it was
sizing for.

The fix for the reported site landed as `target_os_is_hosted()` in
`nros-zpico-build`. This gate exists because ONE fixed site is not the class,
and the class has two failure modes:

  1. **The list goes stale.** A new board lands with a triple whose `target_os`
     is neither `"none"` nor a host — a second NuttX arch, an `*-espidf`
     target — and it silently takes the hosted budget again. Nothing else in
     the tree would notice: the build succeeds, the image runs, the RAM is
     simply gone.

  2. **A second spelling appears.** The question is also asked as a `cfg`
     predicate, where a function cannot be called, so the answer has to be
     re-spelled. It was: `nros-macros` writes
     `not(any(target_os = "none", target_os = "nuttx"))` and
     `nros-rmw-zenoh`'s `effective_client_locator` wrote
     `not(target_os = "none"))` — the same defect, one crate over, in a
     function whose own doc-comment says the default is "deliberately
     HOSTED-only" and that an embedded image dialling `tcp/127.0.0.1:7447` is
     "strictly worse" than no locator at all.

So: check both. Rule 1 resolves every target triple the repository names and
insists its `target_os` is accounted for. Rule 2 insists every `cfg` predicate
that mentions `target_os = "none"` either names every REACHABLE RTOS beside it,
or is classified below with a reason for why `"none"` alone is the right
question there (a bare-metal link section, a `Box`-availability test, an
interrupt-masking critical section — all real, all different from "is this
hosted").

Rule 2 is a ratchet: the exemptions are a fixed table, so a NEW predicate has
to be classified by whoever writes it rather than inheriting silence.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

# ---------------------------------------------------------------------------
# The canonical list lives in the code that uses it, not here.
# ---------------------------------------------------------------------------

RTOS_LIST_FILE = Path("packages/rmw/zenoh/nros-zpico-build/src/runner.rs")
RTOS_LIST_CONST = "RTOS_TARGET_OS"

# `target_os` values that really are hosts. Kept here rather than derived
# because "is this a host" is a claim about the WORLD, not about this tree —
# `rustc --print target-list` cannot tell you which of its targets has a
# filesystem and a process model.
#
# macOS is on this list as a FACT about the target, not as support: nano-ros
# does not ship macOS branches (phase-260 / issue 0916). It is here so that a
# `target_os = "macos"` triple would be classified as a host rather than
# reported as an unknown RTOS.
HOST_TARGET_OS = frozenset(
    {
        "linux",
        "android",
        "macos",
        "ios",
        "windows",
        "freebsd",
        "netbsd",
        "openbsd",
        "dragonfly",
        "solaris",
        "illumos",
        "redox",
        "haiku",
        "fuchsia",
    }
)

# ---------------------------------------------------------------------------
# Rule 2's classified predicates.
#
# Key: (path relative to the repo root, the predicate text with whitespace
# collapsed). A file may legitimately hold several distinct predicates; each
# one is classified on its own, because they are different questions.
#
# Everything here answers something OTHER than "is this target hosted?", which
# is the only question `target_os = "none"` alone gets wrong.
# ---------------------------------------------------------------------------

CLASSIFIED: dict[tuple[str, str], str] = {
    (
        "packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/sync.rs",
        'cfg(target_os = "none")',
    ): "Bare-metal question: `critical_section::Mutex` needs interrupt masking, "
    "which only bare metal provides. The module header states the RTOS answer "
    "explicitly — 'Other (e.g. RTOS targets compiled with their own "
    "`target_os`): falls back to `spin::Mutex`' — and spin is correct there.",
    (
        "packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/sync.rs",
        'cfg(not(target_os = "none"))',
    ): "The other arm of the same bare-metal split.",
    (
        "packages/core/nros-macros/src/main_macro.rs",
        'cfg_attr(target_os = "none", unsafe(link_section = ".nros_boot_config"))',
    ): "Bare-metal question: the boot-config link section exists only in a "
    "bare-metal image's link script. A NuttX image is loaded by NuttX and has "
    "no such section.",
    (
        "packages/core/nros-macros/src/main_macro.rs",
        'cfg(target_os = "none")',
    ): 'Bare-metal question: the OwnedSpin `extern "C" fn main` entry shape '
    "(phase-244 D1) is for images whose C runtime calls `main` directly. NuttX "
    "uses the board-run entry shape and must not take this arm.",
    (
        "packages/rmw/cffi/src/rust_adapter.rs",
        'cfg(all(target_os = "none", not(feature = "std")))',
    ): "Allocator question, not a hosted one: the static subscriber pool exists "
    "because bare metal has no `Box`. NuttX has a global allocator, so the "
    "`Box` arm is the correct one for it.",
    (
        "packages/rmw/cffi/src/rust_adapter.rs",
        'cfg(not(all(target_os = "none", not(feature = "std"))))',
    ): "The other arm of the same allocator split.",
    (
        "packages/rmw/cffi/src/section.rs",
        'cfg(not(target_os = "none"))',
    ): "Deliberate over-inclusion, documented in this module's header: the RTOS "
    "targets DO get the `.init_array` ctor and it is harmless there, because "
    "the board also calls `register()` explicitly and `register()` is "
    "idempotent. Narrowing it would drop a working registration path.",
    (
        "packages/platform/nros-baremetal-common/src/libc_stubs.rs",
        'cfg(all(target_arch = "arm", target_os = "none"))',
    ): "Bare-metal question: these stubs stand in for a libc that is absent "
    "only on bare metal. NuttX supplies its own.",
    (
        "packages/platform/nros-baremetal-common/src/libc_stubs.rs",
        'cfg(not(all(target_arch = "arm", target_os = "none")))',
    ): "The other arm of the same bare-metal split.",
}


# ---------------------------------------------------------------------------
# Rule 1 — every triple this tree names resolves to an accounted-for target_os.
# ---------------------------------------------------------------------------


def rtos_target_os() -> list[str]:
    """The canonical list, read from the crate that owns it."""
    src = (REPO / RTOS_LIST_FILE).read_text(encoding="utf-8")
    m = re.search(
        rf"const\s+{RTOS_LIST_CONST}\s*:\s*&\[&str\]\s*=\s*&\[(?P<body>.*?)\];",
        src,
        re.S,
    )
    if not m:
        raise SystemExit(
            f"check-rtos-target-os: could not find `{RTOS_LIST_CONST}` in "
            f"{RTOS_LIST_FILE}. It is the SSoT for this gate; if it moved, "
            f"point RTOS_LIST_FILE at its new home rather than duplicating it."
        )
    return re.findall(r'"([^"]+)"', m.group("body"))


def git_ls(*pathspecs: str) -> list[str]:
    """Tracked paths matching `pathspecs`, as repo-relative strings.

    `git ls-files` rather than a filesystem walk: an index lookup, not a walk
    that stats every directory it considers pruning (`check-no-tracked-file-find`
    measured 7m36s -> 0.8s for the same 232 paths). Everything this gate reads is
    tracked by definition — a board manifest, a leaf cargo config, a `.rs` file —
    so the index is also the RIGHT set: it excludes `target/`, `build/` and the
    uncommitted `generated/` trees without a filter.
    """
    proc = subprocess.run(
        ["git", "ls-files", "-z", "--", *pathspecs],
        capture_output=True,
        text=True,
        cwd=REPO,
        check=True,
    )
    return [p for p in proc.stdout.split("\0") if p]


def repo_target_triples() -> dict[str, list[str]]:
    """Every Rust target triple the repository names, and where it says it."""
    named = re.compile(r'^\s*target\s*=\s*"([^"]+)"', re.M)
    table = re.compile(r"^\s*\[target\.([^\]]+)\]", re.M)
    out: dict[str, list[str]] = {}
    files = git_ls(
        ":(glob)**/nros-board.toml",
        ":(glob)**/.cargo/config.toml",
    )
    for rel in sorted(set(files)):
        text = (REPO / rel).read_text(encoding="utf-8", errors="replace")
        for m in list(named.finditer(text)) + list(table.finditer(text)):
            triple = m.group(1)
            if triple.endswith(".json"):
                triple = triple[: -len(".json")]
            # `[target.'cfg(...)']` and friends are not triples.
            if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]*", triple):
                continue
            out.setdefault(triple, []).append(rel)
    return out


def target_os_of(triple: str) -> str | None:
    """`target_os` for a triple: from rustc, else from an in-tree target JSON."""
    try:
        proc = subprocess.run(
            ["rustc", "--print", "cfg", "--target", triple],
            capture_output=True,
            text=True,
            cwd=REPO,
        )
    except FileNotFoundError:
        raise SystemExit("check-rtos-target-os: rustc not on PATH")
    if proc.returncode == 0:
        for line in proc.stdout.splitlines():
            if line.startswith("target_os="):
                return line.split("=", 1)[1].strip('"')
        # A target with no `target_os` at all is bare metal by definition.
        return "none"
    for rel in git_ls(f":(glob)**/{triple}.json"):
        spec = json.loads((REPO / rel).read_text(encoding="utf-8"))
        return spec.get("os", "none")
    return None


# ---------------------------------------------------------------------------
# Rule 2 — one spelling for the hosted question.
# ---------------------------------------------------------------------------


def cfg_predicates(text: str) -> list[str]:
    """Every `cfg`/`cfg_attr` predicate in `text` that mentions target_os none.

    Factored out so the selftest can drive it on synthetic input rather than on
    the tree, where "it found nothing" and "there is nothing" look identical.
    """
    found: list[str] = []
    for m in re.finditer(r"#\[(cfg|cfg_attr)\(", text):
        start = m.end() - 1
        depth = 0
        end = start
        for i in range(start, len(text)):
            if text[i] == "(":
                depth += 1
            elif text[i] == ")":
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
        else:
            continue
        pred = f"{m.group(1)}{text[start:end]}"
        if 'target_os = "none"' in pred:
            found.append(re.sub(r"\s+", " ", pred).strip())
    return found


def names_every(pred: str, oses: list[str]) -> bool:
    """Does this predicate name every one of `oses` beside `"none"`?"""
    return all(f'target_os = "{os}"' in pred for os in oses)


def rust_sources() -> list[str]:
    """Every tracked `.rs` under `packages/` and `examples/`, repo-relative."""
    out = []
    for rel in git_ls(":(glob)packages/**/*.rs", ":(glob)examples/**/*.rs"):
        if "/third-party/" in f"/{rel}":
            continue
        out.append(rel)
    return sorted(out)


# ---------------------------------------------------------------------------


def self_test() -> None:
    """Run every time, not behind a flag (`check-gate-selftests`).

    A negative control nobody runs decays into a comment.
    """
    hosted_form = '#[cfg(not(target_os = "none"))]\nfn f() {}'
    assert cfg_predicates(hosted_form) == ['cfg(not(target_os = "none"))'], (
        "selftest: missed the exact predicate issue 1028 was filed against"
    )
    fixed_form = '#[cfg(not(any(target_os = "none", target_os = "nuttx")))]'
    (pred,) = cfg_predicates(fixed_form)
    assert names_every(pred, ["nuttx"]), "selftest: the FIXED spelling read as broken"
    (broken,) = cfg_predicates(hosted_form)
    assert not names_every(broken, ["nuttx"]), (
        "selftest: the BROKEN spelling read as fixed — this gate would pass its "
        "own regression"
    )
    # A predicate about something else must not be picked up at all.
    assert cfg_predicates('#[cfg(feature = "std")]') == [], (
        "selftest: false positive on an unrelated cfg"
    )
    # Nesting: the scanner must take the whole balanced predicate, not stop at
    # the first `)`.
    nested = '#[cfg(all(target_os = "none", not(feature = "std")))]'
    assert cfg_predicates(nested) == [
        'cfg(all(target_os = "none", not(feature = "std")))'
    ], "selftest: unbalanced parse of a nested predicate"
    # Rule 1's classifier must reject an OS it has never heard of.
    unknown = "vxworks"
    assert unknown not in HOST_TARGET_OS, "selftest: fixture OS is in the host set"


def main() -> int:
    rtos = rtos_target_os()
    triples = repo_target_triples()

    failures: list[str] = []
    reachable: set[str] = set()
    unresolved: list[str] = []

    for triple, where in sorted(triples.items()):
        os_name = target_os_of(triple)
        if os_name is None:
            unresolved.append(f"  {triple} (named by {where[0]})")
            continue
        if os_name == "none" or os_name in HOST_TARGET_OS:
            continue
        if os_name in rtos:
            reachable.add(os_name)
            continue
        failures.append(
            f"  {triple} -> target_os = {os_name!r}\n"
            f"      named by {where[0]}\n"
            f'      Not "none", not a host, and not in {RTOS_LIST_CONST}. It will\n'
            f"      therefore be sized as a Linux-class host. Add it to\n"
            f"      {RTOS_LIST_FILE}:{RTOS_LIST_CONST} if it is an RTOS, or to\n"
            f"      HOST_TARGET_OS in this script if it really is a host."
        )

    if failures:
        print("check-rtos-target-os: a target this tree builds is classified wrong\n")
        print("\n".join(failures))
        print(
            "\nIssue 1028: an RTOS that names its own `target_os` takes the hosted\n"
            "budget. Measured at 106,752 B of `.bss` on one NuttX image."
        )
        return 1

    reachable_sorted = sorted(reachable)
    spelling_failures: list[str] = []
    seen_classified: set[tuple[str, str]] = set()

    for rel in rust_sources():
        text = (REPO / rel).read_text(encoding="utf-8", errors="replace")
        if 'target_os = "none"' not in text:
            continue
        for pred in dict.fromkeys(cfg_predicates(text)):
            key = (rel, pred)
            if key in CLASSIFIED:
                seen_classified.add(key)
                continue
            if names_every(pred, reachable_sorted):
                continue
            spelling_failures.append(
                f"  {rel}\n"
                f"      {pred}\n"
                f'      Mentions `target_os = "none"` without naming '
                f"{', '.join(reachable_sorted)}."
            )

    stale = sorted(set(CLASSIFIED) - seen_classified)
    if stale:
        print(
            "check-rtos-target-os: a classified predicate no longer exists\n\n"
            "A stale exemption is the issue-0743 class: it reads as coverage and\n"
            "checks nothing. Delete the row whose site is gone.\n"
        )
        for rel, pred in stale:
            print(f"  {rel}\n      {pred}")
        return 1

    if spelling_failures:
        print(
            "check-rtos-target-os: a `cfg` asks the hosted question with the\n"
            "predicate issue 1028 measured wrong\n"
        )
        print("\n".join(spelling_failures))
        print(
            f'\n`target_os = "none"` alone means BARE METAL. The RTOSes this tree\n'
            f"reaches that name themselves are: {', '.join(reachable_sorted)}.\n"
            f"If the predicate asks 'is this hosted?', name them beside \"none\":\n"
            f'  #[cfg(not(any(target_os = "none", target_os = "nuttx")))]\n'
            f"If it asks something else — is there a libc, can I `Box`, is there a\n"
            f"link section — add it to CLASSIFIED in this script WITH THE REASON."
        )
        return 1

    note = ""
    if unresolved:
        note = f", {len(unresolved)} triple(s) unresolvable by this toolchain"
    print(
        f"check-rtos-target-os: OK - {len(triples)} triple(s) classified"
        f"{note}; RTOS reachable here: {', '.join(reachable_sorted) or '(none)'}; "
        f"{len(CLASSIFIED)} classified predicate(s)"
    )
    if unresolved:
        print(
            "  (unresolvable means neither rustc nor an in-tree target JSON knows\n"
            "  the triple — reported, not failed, because a missing toolchain is\n"
            "  not a wrong classification.)"
        )
        print("\n".join(unresolved))
    return 0


if __name__ == "__main__":
    # Normal path, every run.
    self_test()
    sys.exit(main())
