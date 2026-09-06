#!/usr/bin/env python3
"""An `[arch.*]` profile must be REACHABLE, UNIQUE, and wired to the right ABI.

phase-418 418.3. This is phase-385 W1's defect turned into a check, plus the
two neighbours that defect had.

WHAT PHASE-385 W1 FOUND, AND WHY A GATE
---------------------------------------
`[arch.cortex-r52]` landed with phase-372 W1 and was never added to the
platform's `arch = [..]` list. The resolver
(`nros_board_common::arch_flags::cflags_for_target`) walks THAT list, in
declared order, first match wins — so the block was unreachable and an
`armv8r-none-eabihf` cargo build panicked with "no [arch.*] profile … admits
TARGET", naming only m3 and m7. The S32Z270 board never noticed, because its
CMake lane passes `FREERTOS_CFLAGS` explicitly and short-circuits the lookup.

That is this repo's most-repeated shape: a declaration that reads as coverage
and reaches nothing. It was found by building, months later, on the one lane
that could not short-circuit. Reading the manifest answers it in milliseconds.

THE THIRD RULE IS THE ONE THAT EARNS ITS KEEP
---------------------------------------------
Reachability alone would have passed a profile that resolves and is WRONG. The
predicates are SUBSTRING tests on the triple (`arch_matches`), and float ABI is
spelled in two independent places — the triple's `eabi`/`eabihf` suffix, chosen
by the Rust side, and `-mfloat-abi=` in the profile's `cflags`, chosen by the C
side. Nothing connects them. Get them wrong in the same direction and
everything links; the first `float` across the boundary is read from the wrong
register file, because `hard` is AAPCS-VFP (s0../d0..) and `soft`/`softfp` is
AAPCS base (r0..).

phase-418 418.3 is exactly that hazard: the NVIDIA Orin SPE FSP compiles AND
links `-mfloat-abi=softfp`, while `rust-toolchain.toml` has carried
`armv7r-none-eabihf` for this target since phase-100. And `"armv7r-none-eabi"`
is a SUBSTRING of `"armv7r-none-eabihf"`, so a `[arch.cortex-r5]` written
without `target_exclude = "eabihf"` claims the hard-float triple too — a
shape-valid profile, resolving happily, wired to the wrong ABI.

WHAT THE MISMATCH ACTUALLY COSTS — MEASURED, NOT ASSUMED
--------------------------------------------------------
GNU ld does catch it, through the EABI attribute `Tag_ABI_VFP_args`. Measured
with the pinned `arm-none-eabi-gcc 13.2.1` (phase-418 418.3): a `softfp` C
object linked against an `armv7r-none-eabihf` Rust staticlib gives

    ld: error: librust_hf.a(...rcgu.o) uses VFP register arguments, <out> does
        not
    ld: failed to merge target specific data of file ...

and the matching `armv7r-none-eabi` + `-Ctarget-feature=+vfp3d16` archive links
clean. So the claim to make here is NOT "it links and reads garbage" — it is
that the diagnosis arrives at the END of a long cross build, names a mangled
`rcgu.o` inside an rlib, and says nothing about the profile that chose the
flags. That is the same distance-from-cause phase-385 W1's panic had, and the
same reason to answer it from the manifest in milliseconds. (`soft` and
`softfp` both set `Tag_ABI_VFP_args = 0`, so they merge — the only mismatch
that exists is hard-vs-not, which is exactly what this rule tests.)

So rule 3 asks the question the two spellings cannot ask of each other:
**for every triple a profile actually claims, does its `-mfloat-abi=` agree
with the triple's own ABI?**

Deliberately one-directional. A missing `-mfloat-abi=` is legal — `cortex-a7`
under NuttX carries only `-march=armv7-a` and lets the cross compiler's default
stand, which is a real and working configuration. What is never legal is a
profile that STATES an ABI contradicting the triple it claims.

THE RULES
---------
  R1  every name in a platform's `arch = [..]` has an `[arch.<name>]` block
      readable from that platform's root                     (listed => defined)
  R2  every `[arch.<name>]` block is named by some platform's `arch = [..]`
      in that root                                           (defined => listed)
  R3  for every (platform, triple) the resolver would answer, the resolved
      `-mfloat-abi=` agrees with the triple's ABI suffix      (the wiring)
  R4  at most one of a platform's declared profiles admits a triple — two is
      an answer decided silently by list order                (no shadowing)
  R5  every `[arch.<name>]` block admits at least one triple in the universe
      below                                                  (nothing inert)

R5 IS R2 FROM THE OTHER SIDE, AND R1-R4 ALL PASS WITHOUT IT
-----------------------------------------------------------
Found by review of the first version of this file, with a mutation every other
rule blesses:

    [arch.cortex-r5]
    target_match = "armv8r"        # was "armv7r-none-eabi"
    target_exclude = "eabihf"
    cflags = [.., "-mfloat-abi=softfp", ..]

Well-formed, defined, listed, and it matches NOTHING: `armv8r-none-eabihf`
contains `eabihf`, so its own exclude vetoes its own match. R1 holds, R2 holds,
R4 holds vacuously, and R3 cannot fire because a rule about (profile, triple)
pairs has no pair to judge. The gate said OK.

phase-385 W1 was "a triple with no profile", which PANICS and is therefore
eventually found. This is "a profile with no triple", which is silently inert —
and produces the identical downstream failure, because the ARMv7-R build again
resolves nothing. A fat-fingered `target_match`, or a triple renamed in
`config/rust-targets.txt` with the profile not following, both land exactly
here. A rule whose coverage is narrower than the fault it names is this repo's
most-repeated defect (issue 0196's class); R5 closes that side.

TWO DEBT LISTS, NOT ONE
-----------------------
R2 and R5 are different defects with different remedies — "nobody lists it"
is fixed by editing an `arch = [..]`, "it matches nothing" by fixing a
`target_match` or provisioning a triple — so `KNOWN_UNLISTED` and
`KNOWN_INERT` are separate, and a profile that is both appears in both.
`config/bare-metal`'s `riscv32gc` is exactly that. Merging them would mean
fixing one half silently keeps suppressing the other, which is the shape of
allowlist that stops being a ratchet. Two self-test cases pin the
non-suppression in both directions.

The triple universe is `config/rust-targets.txt`, the tree's ONE list of Rust
targets (issue 0833). That is deliberate: a profile is only interesting for a
triple somebody provisions, and reusing the list means a new triple is checked
against every profile the day it is provisioned, with nothing to remember.

WHAT IT DOES NOT DO
-------------------
`inherits` is not followed. No in-tree platform file uses it, and rather than
under-approximate silently the moment one does, a file that declares it is a
hard ERROR here naming this paragraph. Same reason `arch_matches` is
re-implemented rather than shelled out to: the Rust helper is the SSoT for
BEHAVIOUR and this file is 8 lines of substring test, but the check must run in
the fast lane with no cargo. The re-implementation is pinned by
`predicate_matches_the_rust_helper` in the self-test, which encodes the Rust
doc-comment's own examples.
"""

from __future__ import annotations

import io
import contextlib
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ROOTS = ("config", "packages/platform")
TARGET_LIST = ROOT / "config" / "rust-targets.txt"

# R2 debt: defined, and no platform lists it. NOT a general exemption — an
# unreachable profile is still a defect; these are its pre-existing instances,
# found BY this gate on the day it was written, in a file phase-418 418.3 does
# not own.
#
# `config/bare-metal` declares `arch = ["cortex-m3", "riscv32imc"]` while
# defining four profiles. So a bare-metal `thumbv7em-none-eabihf` (STM32F4) or
# `riscv32gc` build resolves NOTHING and panics — phase-385 W1's defect,
# untouched, two more times. Fixing it means editing that platform's list,
# which is a behaviour change for those boards and belongs to whoever owns
# them.
# Each row names a TRACKED issue, not just a reason: a gap that is real but
# open is tolerable, an unattributed one is a silent exemption (CLAUDE.md, the
# rmw-parity rule).
KNOWN_UNLISTED = {
    ("config/bare-metal", "cortex-m4f"):
        "issue 1154 — bare-metal lists only cortex-m3 + riscv32imc",
    ("config/bare-metal", "riscv32gc"):
        "issue 1154 — bare-metal lists only cortex-m3 + riscv32imc",
}

# R5 debt: defined, and it admits no triple anybody provisions. A DIFFERENT
# fault from the list above with a DIFFERENT remedy, so a separate list —
# `riscv32gc` is in both because it is both, and removing it from one must not
# stop the other from being reported.
#
# `[arch.riscv32gc]` matches the substring `riscv32gc`, which appears in no row
# of `config/rust-targets.txt` (the provisioned RISC-V triples are
# `riscv32imc-unknown-none-elf`, `riscv64gc-unknown-none-elf` and
# `riscv32imac-unknown-nuttx-elf`). So even once bare-metal lists it, it still
# resolves for nothing.
KNOWN_INERT = {
    ("config/bare-metal", "riscv32gc"):
        "issue 1154 (second fault) — no provisioned triple contains `riscv32gc`, "
        "so listing it is necessary and NOT sufficient",
}

# Triples whose ABI this gate can read off the name. Only the ARM EABI family
# spells float ABI in the triple; `riscv*` encodes it in `-mabi=` on both sides
# and `*-unknown-none-elf` says nothing, so those are skipped rather than
# guessed at.
ARM_EABI = re.compile(r"^(?:armv?|thumbv)[0-9]")


def parse_platform_file(text: str) -> tuple[dict, list[str], bool]:
    """(`[arch.*]` blocks, the `arch = [..]` names, declares `inherits`).

    Regex rather than a TOML library: this runs in the fast lane on whatever
    Python the host has, and 3.10 has no `tomllib`. `check-rust-targets-covered`
    parses `nros-sdk-index.toml` the same way and for the same reason.
    """
    profiles: dict[str, dict] = {}
    declared: list[str] = []
    inherits = False

    section = ""          # current table path, e.g. "arch.cortex-r5"
    current: dict | None = None
    pending_key = None    # a `key = [` whose array spans lines
    pending: list[str] = []

    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip() if not raw.strip().startswith("#") else ""
        if not line:
            continue
        if line.startswith("["):
            section = line.strip("[]").strip()
            current = None
            pending_key = None
            if section.startswith("arch."):
                current = profiles.setdefault(section[len("arch."):], {})
            continue

        if pending_key is not None:
            pending.append(line)
            if "]" in line:
                value = " ".join(pending)
                if pending_key == "arch":
                    declared.extend(re.findall(r'"([^"]+)"', value))
                elif current is not None:
                    current[pending_key] = re.findall(r'"([^"]+)"', value)
                pending_key = None
            continue

        m = re.match(r'^([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*)$', line)
        if not m:
            continue
        key, value = m.group(1), m.group(2).strip()

        # `inherits` only counts at the file's top level.
        if key == "inherits" and section == "" and value not in ('""', "''"):
            inherits = True

        # The `arch = ..` LIST lives under `[build.<component>]`; the `[arch.*]`
        # BLOCKS are top-level tables. Two different things, one word.
        if key == "arch" and section.startswith("build"):
            if value.startswith("[") and "]" not in value:
                pending_key, pending = "arch", [value]
            elif value.startswith("["):
                declared.extend(re.findall(r'"([^"]+)"', value))
            else:
                declared.extend(re.findall(r'"([^"]+)"', value))
            continue

        if current is None:
            continue
        if key in ("target_match", "target_exclude"):
            got = re.findall(r'"([^"]*)"', value)
            if got:
                current[key] = got[0]
        elif key == "cflags":
            if value.startswith("[") and "]" not in value:
                pending_key, pending = "cflags", [value]
            else:
                current["cflags"] = re.findall(r'"([^"]+)"', value)

    return profiles, declared, inherits


def arch_matches(profile: dict, target: str) -> bool:
    """Mirror of `nros_board_common::arch_flags::arch_matches`.

    Both predicates are SUBSTRING tests: `target_match` must be present in the
    triple, `target_exclude` must not. Pinned by the self-test.
    """
    needle = profile.get("target_match")
    if needle is not None and needle not in target:
        return False
    veto = profile.get("target_exclude")
    if veto is not None and veto in target:
        return False
    return True


def triple_abi(triple: str) -> str | None:
    """`hard`, `soft`, or None when the triple does not spell an ABI."""
    if not ARM_EABI.match(triple):
        return None
    stem = triple.split(".json")[0]
    if stem.endswith("hf"):
        return "hard"
    if re.search(r"eabi$", stem) or "-eabi-" in stem:
        return "soft"
    return None


def cflags_abi(cflags: list[str]) -> str | None:
    """`hard`, `soft`, or None when the profile states no float ABI.

    `softfp` is a CODEGEN choice (emit VFP instructions) on top of the SOFT
    calling convention, so it groups with `soft` here — which is the whole
    point: the SPE FSP is `softfp` and therefore soft-ABI.
    """
    abi = None
    for f in cflags:
        m = re.match(r"^-mfloat-abi=(\w+)$", f)
        if m:
            abi = "hard" if m.group(1) == "hard" else "soft"
    return abi


def listed_triples() -> list[str]:
    out = []
    for raw in TARGET_LIST.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        out.append(line.split()[0])
    return out


def load_tree() -> dict:
    """{root: {platform_rel: (profiles, declared, inherits)}}, from disk."""
    tree: dict[str, dict] = {}
    for root in ROOTS:
        base = ROOT / root
        if not base.is_dir():
            continue
        for f in sorted(base.glob("*/nros-platform.toml")):
            rel = f"{root}/{f.parent.name}"
            tree.setdefault(root, {})[rel] = parse_platform_file(
                f.read_text(encoding="utf-8")
            )
    return tree


def debt_is_live(tree: dict, triples: list[str], unlisted: dict,
                 inert: dict) -> list[str]:
    """Every allowlist entry must be load-bearing FOR ITS OWN RULE.

    Added after a mutation the rest of this file survived. `check()` consults
    the two lists correctly; `main()` can still hand it the wrong ones —

        merged = {**KNOWN_UNLISTED, **KNOWN_INERT}
        check(tree, triples, merged, merged)

    — and every rule above stays green, because each list is still *a* dict
    with plausible keys. That is the same class as M6 (a profile wired to the
    wrong arch) one level up: the records are fine, the wiring is not. Nothing
    that only reads `check()` can see it.

    So this asks a question that has a different answer for each list: remove
    one entry, and does the fault it excuses actually appear, with ITS rule's
    message? Merging the lists makes `cortex-m4f` an R5 exemption, and
    `cortex-m4f` admits `thumbv7em-none-eabihf` perfectly well, so it excuses
    nothing and is reported here.

    Second thing it buys, for free: both lists become self-cleaning. An entry
    whose fault someone fixed stops being load-bearing and has to be deleted,
    so the debt can only shrink — the ratchet shape `check-gate-selftests` and
    `check-gate-visibility` both use.

    It takes the values `main()` PASSES, never the module constants, because
    reading the constants is exactly the blindness being closed.
    """
    problems = []
    for label, table, other, marker, rule in (
        ("KNOWN_UNLISTED", unlisted, inert, "is defined and NO platform", "R2"),
        ("KNOWN_INERT", inert, unlisted, "admits NO triple", "R5"),
    ):
        for key in sorted(table):
            reduced = {k: v for k, v in table.items() if k != key}
            args = ((reduced, other) if label == "KNOWN_UNLISTED"
                    else (other, reduced))
            buf = io.StringIO()
            with contextlib.redirect_stderr(buf):
                check(tree, triples, *args)
            if f"[arch.{key[1]}] {marker}" not in buf.getvalue():
                problems.append(
                    f"{label}[{key[0]}, {key[1]}] excuses nothing.\n"
                    f"      Removing it does not produce a {rule} failure, so it "
                    f"is either stale — the\n"
                    f"      fault was fixed and the entry outlived it — or it is "
                    f"in the WRONG LIST,\n"
                    f"      which is what merging the two debt lists looks like "
                    f"from here. Delete it,\n"
                    f"      or move it to the list for the rule it actually "
                    f"excuses."
                )
    return problems


def check(tree: dict, triples: list[str], known_unlisted: dict,
          known_inert: dict) -> int:
    problems: list[str] = []

    for root, files in tree.items():
        # `[arch.*]` merges across every file in a root (`PlatformsTree`'s
        # merged arch table), so reachability is a per-ROOT question, not a
        # per-file one.
        merged: dict[str, dict] = {}
        owner: dict[str, str] = {}
        for rel, (profiles, _, _) in files.items():
            for name, prof in profiles.items():
                merged[name] = prof
                owner.setdefault(name, rel)

        claimed: set[str] = set()

        for rel, (_, declared, inherits) in sorted(files.items()):
            if inherits:
                problems.append(
                    f"{rel}: declares `inherits`, which this gate does not follow.\n"
                    f"      Extend the resolution here before landing it — the\n"
                    f"      alternative is a check that silently stops seeing the\n"
                    f"      profiles a platform reaches through its parent."
                )
                continue
            claimed.update(declared)

            # R1 — listed => defined.
            for name in declared:
                if name not in merged:
                    problems.append(
                        f"{rel}: `arch = [..]` names `{name}`, but no "
                        f"[arch.{name}] block exists under {root}/.\n"
                        f"      `cflags_for_target` returns Err for this and the "
                        f"build stops with it."
                    )

            # R3 / R4 — resolve each triple exactly as the Rust helper does.
            for triple in triples:
                hits = [n for n in declared
                        if n in merged and arch_matches(merged[n], triple)]
                if not hits:
                    continue
                if len(hits) > 1:
                    problems.append(
                        f"{rel}: {triple} is admitted by {len(hits)} profiles "
                        f"({', '.join(hits)}).\n"
                        f"      First-match-wins picks `{hits[0]}` by LIST ORDER "
                        f"alone. Narrow one\n"
                        f"      with `target_exclude`, or make the `target_match` "
                        f"specific enough."
                    )
                want = triple_abi(triple)
                got = cflags_abi(merged[hits[0]].get("cflags", []))
                if want and got and want != got:
                    problems.append(
                        f"{rel}: [arch.{hits[0]}] claims {triple} "
                        f"({want}-float ABI) but its cflags say "
                        f"-mfloat-abi={'hard' if got == 'hard' else 'soft/softfp'}.\n"
                        f"      hard is AAPCS-VFP (floats in s0../d0..), soft and "
                        f"softfp are AAPCS base\n"
                        f"      (floats in r0..). ld rejects the pair late and "
                        f"obscurely — \"uses VFP\n"
                        f"      register arguments\", naming an rcgu.o inside an "
                        f"rlib, at the end of a\n"
                        f"      cross build, saying nothing about the profile that "
                        f"chose the flags.\n"
                        f"      Either pick the other triple, or exclude it here "
                        f"(`target_exclude`) —\n"
                        f"      the predicates are SUBSTRING tests, so `-none-eabi` "
                        f"claims `-none-eabihf`\n"
                        f"      unless you say otherwise."
                    )

        # R2 — defined => listed, per root. R5 — defined => admits something.
        # Independent faults, reported independently: a profile can be both,
        # and fixing either alone leaves a build resolving nothing.
        for name in sorted(merged):
            if name not in claimed and (owner[name], name) not in known_unlisted:
                problems.append(
                    f"{owner[name]}: [arch.{name}] is defined and NO platform "
                    f"under {root}/ lists it\n"
                    f"      in `arch = [..]`. The resolver walks that list, so "
                    f"the block is unreachable:\n"
                    f"      a build for its target panics \"no [arch.*] profile … "
                    f"admits TARGET\" while the\n"
                    f"      profile sits right there. This is phase-385 W1's "
                    f"defect."
                )
            if (not any(arch_matches(merged[name], tr) for tr in triples)
                    and (owner[name], name) not in known_inert):
                problems.append(
                    f"{owner[name]}: [arch.{name}] admits NO triple in "
                    f"config/rust-targets.txt.\n"
                    f"      match={merged[name].get('target_match')!r} "
                    f"exclude={merged[name].get('target_exclude')!r} — both are "
                    f"SUBSTRING tests,\n"
                    f"      and an exclude that appears in its own match's "
                    f"triples vetoes the profile\n"
                    f"      entirely. The block is INERT: every other rule here "
                    f"passes (it is listed,\n"
                    f"      defined, shadows nothing, and has no pair for the "
                    f"ABI rule to judge) while a\n"
                    f"      build for its arch resolves nothing — phase-385 W1's "
                    f"defect from the other\n"
                    f"      side. Fix the predicate, or provision the triple in "
                    f"config/rust-targets.txt."
                )

    if problems:
        print("check-arch-profile-resolution: FAILED\n", file=sys.stderr)
        for p in problems:
            print(f"  {p}\n", file=sys.stderr)
        return 1
    return 0


def _selftest_tree(**over) -> dict:
    """A two-platform root shaped like the real one, for the negative controls."""
    r5 = {
        "target_match": "armv7r-none-eabi",
        "target_exclude": "eabihf",
        "cflags": ["-mcpu=cortex-r5", "-mfloat-abi=softfp", "-mfpu=vfpv3-d16"],
    }
    r52 = {"target_match": "armv8r",
           "cflags": ["-mcpu=cortex-r52", "-mfloat-abi=hard"]}
    r5.update(over.pop("r5", {}))
    r52.update(over.pop("r52", {}))
    declared = over.pop("declared", ["cortex-r5", "cortex-r52"])
    inherits = over.pop("inherits", False)
    assert not over, over
    return {"t": {"t/p": ({"cortex-r5": r5, "cortex-r52": r52}, declared, inherits)}}


def self_test() -> int:
    """Runs on the NORMAL path, never behind a flag — `check-gate-selftests`.

    A negative control nobody runs decays into a comment. The cases below are
    the four rules plus the mutation class that matters most here: a profile
    whose SHAPE is perfectly valid and whose WIRING is wrong.
    """
    ok = True
    triples = ["armv7r-none-eabi", "armv7r-none-eabihf", "armv8r-none-eabihf",
               "riscv32imc-unknown-none-elf"]

    def case(name, tree, want_rc, expect_text=None, known=None, inert=None):
        nonlocal ok
        buf = io.StringIO()
        with contextlib.redirect_stderr(buf):
            rc = check(tree, triples, known or {}, inert or {})
        out = buf.getvalue()
        if rc != want_rc:
            ok = False
            print(f"  self-test FAIL {name}: rc={rc} want {want_rc}\n{out}")
        elif expect_text and expect_text not in out:
            ok = False
            print(f"  self-test FAIL {name}: message lacked {expect_text!r}\n{out}")

    # The shipped shape passes.
    case("shipped shape", _selftest_tree(), 0)

    # R1 — a name with no block.
    case("listed but undefined",
         _selftest_tree(declared=["cortex-r5", "cortex-r52", "cortex-r7"]),
         1, "no [arch.cortex-r7] block")

    # R2 — phase-385 W1's own defect: a block nobody lists.
    case("defined but unlisted", _selftest_tree(declared=["cortex-r52"]),
         1, "phase-385 W1's defect")

    # R3, the wiring mutation. SHAPE stays valid — a real flag, a real triple,
    # a profile that resolves — and only the pairing is wrong. This is the
    # mutation a structure-only gate passes.
    case("softfp profile claiming the hard-float triple",
         _selftest_tree(r5={"target_exclude": None}),
         1, "-mfloat-abi=soft/softfp")
    case("hard-float flags on the soft-float triple",
         _selftest_tree(r5={"cflags": ["-mcpu=cortex-r5", "-mfloat-abi=hard"]}),
         1, "-mfloat-abi=hard")

    # R4 — two profiles admitting one triple, order deciding silently.
    case("shadowed triple",
         _selftest_tree(r52={"target_match": "armv", "cflags": []}),
         1, "admitted by 2 profiles")

    # A stated-nothing profile is legal (nuttx `cortex-a7`), so R3 must not
    # fire on absence — otherwise the gate lands red on the shipped tree.
    case("no float ABI stated is legal",
         _selftest_tree(r5={"cflags": ["-mcpu=cortex-r5"]}), 0)

    # R5 — the reviewer's mutation, verbatim: give cortex-r5 the r52 triple and
    # leave its own exclude in place. Every block well-formed, both listed, and
    # `armv8r-none-eabihf` contains `eabihf`, so the profile's own veto kills
    # its own match and it admits nothing. R1/R2/R4 hold; R3 has no pair to
    # judge. This case is the negative control for the whole rule.
    case("profile whose exclude vetoes its own match",
         _selftest_tree(r5={"target_match": "armv8r"}),
         1, "admits NO triple")

    # R5 — the plainer form: a target_match nobody provisions (a rename in
    # config/rust-targets.txt that the profile did not follow).
    case("target_match matches no provisioned triple",
         _selftest_tree(r5={"target_match": "armv6r-none-eabi"}),
         1, "admits NO triple")

    # Two debt lists, and neither may cover for the other. Both directions,
    # because a merged allowlist reads correct until one half is fixed.
    case("known-unlisted is tolerated",
         _selftest_tree(declared=["cortex-r5"]), 0,
         known={("t/p", "cortex-r52"): "test"})
    case("known-unlisted does not suppress a wiring fault",
         _selftest_tree(declared=["cortex-r5"], r5={"target_exclude": None}), 1,
         known={("t/p", "cortex-r52"): "test"})
    case("known-unlisted does not suppress an INERT profile",
         _selftest_tree(declared=["cortex-r5"], r52={"target_match": "nope"}), 1,
         known={("t/p", "cortex-r52"): "test"})
    case("known-inert is tolerated",
         _selftest_tree(r5={"target_match": "nope"}), 0,
         inert={("t/p", "cortex-r5"): "test"})
    case("known-inert does not suppress an UNLISTED profile",
         _selftest_tree(declared=["cortex-r52"], r5={"target_match": "nope"}), 1,
         "phase-385 W1's defect", inert={("t/p", "cortex-r5"): "test"})

    # A profile that is BOTH needs BOTH entries — this is `riscv32gc` in the
    # shipped tree, and it is why the lists are not merged.
    case("both faults need both entries", _selftest_tree(
             declared=["cortex-r52"], r5={"target_match": "nope"}), 0,
         known={("t/p", "cortex-r5"): "test"},
         inert={("t/p", "cortex-r5"): "test"})

    # `debt_is_live` — the wiring check. Its own negative controls, because it
    # exists precisely because the rules above could not see a wiring fault.
    def debt_case(name, tree, unlisted, inert, want_problems, expect=None):
        nonlocal ok
        got = debt_is_live(tree, triples, unlisted, inert)
        if (len(got) > 0) != want_problems:
            ok = False
            print(f"  self-test FAIL debt/{name}: {got}")
        elif expect and not any(expect in g for g in got):
            ok = False
            print(f"  self-test FAIL debt/{name}: lacked {expect!r}: {got}")

    # cortex-r52 unlisted (a real R2 fault) — its entry IS load-bearing.
    debt_case("live R2 entry", _selftest_tree(declared=["cortex-r5"]),
              {("t/p", "cortex-r52"): "x"}, {}, False)
    # cortex-r5 inert (a real R5 fault) — its entry IS load-bearing.
    debt_case("live R5 entry", _selftest_tree(r5={"target_match": "nope"}),
              {}, {("t/p", "cortex-r5"): "x"}, False)
    # An entry excusing nothing: the shipped shape has no fault at all, so any
    # allowlist row over it is stale and must be deleted.
    debt_case("stale entry", _selftest_tree(),
              {("t/p", "cortex-r5"): "x"}, {}, True, "excuses nothing")
    # THE MUTATION THIS FUNCTION EXISTS FOR: the two lists merged. cortex-r52
    # is a genuine R2 exemption and a bogus R5 one; the R5 side is reported.
    both = {("t/p", "cortex-r52"): "x"}
    debt_case("merged debt lists", _selftest_tree(declared=["cortex-r5"]),
              both, both, True, "KNOWN_INERT[t/p, cortex-r52]")
    # An entry in the wrong list outright.
    debt_case("entry in the wrong list", _selftest_tree(declared=["cortex-r5"]),
              {}, {("t/p", "cortex-r52"): "x"}, True, "KNOWN_INERT")

    # `inherits` must be loud, not silently under-resolved.
    case("inherits is refused", _selftest_tree(inherits=True), 1,
         "does not follow")

    # The predicate mirror: the examples from `arch_matches`'s own Rust
    # doc-comment, so a drift in the copy shows up here rather than as a build
    # that resolves differently from the gate that blessed it.
    for prof, target, want in [
        ({"target_match": "thumbv7m", "target_exclude": "thumbv7em"},
         "thumbv7m-none-eabi", True),
        ({"target_match": "thumbv7m", "target_exclude": "thumbv7em"},
         "thumbv7em-none-eabihf", False),
        ({"target_match": "armv7r-none-eabi", "target_exclude": "eabihf"},
         "armv7r-none-eabihf", False),
        ({"target_match": "armv7r-none-eabi"}, "armv7r-none-eabihf", True),
        ({}, "anything", True),
    ]:
        if arch_matches(prof, target) != want:
            ok = False
            print(f"  self-test FAIL predicate: {prof} vs {target} "
                  f"!= {want}")

    # The TOML reader has to tell the `arch = [..]` LIST from the `[arch.*]`
    # BLOCKS — one word, two meanings, and getting it wrong makes every
    # platform read as declaring nothing (R2 red everywhere) or as declaring
    # every profile (R2 vacuous).
    profiles, declared, inh = parse_platform_file(
        '[build.zenoh]\n'
        'arch = ["a", "b"]\n'
        '[arch.a]\n'
        'target_match = "x"\n'
        'cflags = [\n  "-p",\n  "-mfloat-abi=hard",\n]\n'
        '[arch.b]\n'
        'target_match = "y"\n'
    )
    if (sorted(profiles) != ["a", "b"] or declared != ["a", "b"] or inh
            or profiles["a"]["cflags"] != ["-p", "-mfloat-abi=hard"]):
        ok = False
        print(f"  self-test FAIL parser: {profiles} {declared} {inh}")

    print("check-arch-profile-resolution --self-test: "
          + ("OK" if ok else "FAILED"))
    return 0 if ok else 1


def main() -> int:
    if self_test() != 0:
        return 1
    tree = load_tree()
    if not tree:
        print("check-arch-profile-resolution: no platform descriptor root found "
              f"under {', '.join(ROOTS)}", file=sys.stderr)
        return 2
    triples = listed_triples()
    if not triples:
        print("check-arch-profile-resolution: config/rust-targets.txt lists no "
              "targets — the triple universe is empty and every rule below is "
              "vacuous", file=sys.stderr)
        return 2
    stale = debt_is_live(tree, triples, KNOWN_UNLISTED, KNOWN_INERT)
    if stale:
        print("check-arch-profile-resolution: FAILED (debt allowlist)\n",
              file=sys.stderr)
        for s in stale:
            print(f"  {s}\n", file=sys.stderr)
        return 1
    rc = check(tree, triples, KNOWN_UNLISTED, KNOWN_INERT)
    if rc == 0:
        n = sum(len(p[0]) for f in tree.values() for p in f.values())
        print(f"check-arch-profile-resolution: OK ({n} profile(s) across "
              f"{sum(len(f) for f in tree.values())} platform file(s), "
              f"{len(triples)} triple(s), {len(KNOWN_UNLISTED)} known "
              f"unlisted, {len(KNOWN_INERT)} known inert)")
    return rc


if __name__ == "__main__":
    sys.exit(main())
