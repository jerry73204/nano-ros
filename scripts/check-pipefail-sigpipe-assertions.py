#!/usr/bin/env python3
"""A matcher-predicate must be fed a HERE-STRING, never a pipe.  Issue 1077.

    if ! printf '%s' "$out" | grep -q "status=refused"; then

Bash's builtin `printf` flushes PER LINE, and a `-q` grep exits at the FIRST
match.  So whenever the needle is on any line but the last, grep can close the
read end while the writer still has a line to send; the writer takes SIGPIPE,
and `set -o pipefail` promotes that 141 to the pipeline's status.  `if ! …`
then reads a SUCCESSFUL match as a MISS.  Traced on 2026-09-05:

    write(1, "    demo/msg/Open (unbounded)\\n", 30) = -1 EPIPE (Broken pipe)
    --- SIGPIPE ---  +++ killed by SIGPIPE +++     PIPESTATUS=(141 0)

Measured at 60 of 3000 calls (2%) on a three-line haystack on an IDLE 12-core
host — and 13 of 300 runs of the gate that carried it, against 0 of 300 for the
here-string.  That was `check-fast` flaking on four different gates over four
runs, every flake wearing a considered assertion message that pointed at real
code: "a subscribed type with no bound did NOT refuse", "first pass did not
read the placeholder".  Issue 0876's shape — a red that is not a verdict, and
articulate enough to survive a reviewer's judgement.

SIBLING, NOT DUPLICATE.  `check-grep-q-error-conflation` (issue 0726) is about
the same call's EXIT STATUS: grep answers 0 / 1 / >=2 and `if ! …` folds the
error into "no match", so that gate routes the sites through `nros_grep_q`.
It explicitly skips any line naming `nros_grep_q` — which is right for the
status question and blind to this one, because

    printf '%s' "$out" | nros_grep_q PAT

still puts a writer behind a reader that exits early, and still returns 141.
Worse, the helper's fatal path (`exit 2` on a tool failure) runs in the
PIPELINE'S SUBSHELL and cannot stop the caller.  So the two rules are
independent: use `nros_grep_q`, AND feed it `<<<`.

WHAT IS FLAGGED: a pipeline whose LAST stage is a matcher predicate that can
stop reading before EOF (`grep -q`, `grep -m N`, `nros_grep_q`), in a position
where the pipeline's STATUS is read (`if`/`elif`/`while`/`until`/`!`/`&&`/
`; then`/`|| continue`), in a file that enables `pipefail`.

Two shapes are deliberately NOT flagged.  A pipeline in VALUE position
(`x="$(a | head -1)"`) discards its status, so the value it produces is
unaffected; `head` is only a hazard when a predicate reads its status, and
`… | head -1 | grep -q X` is caught by its last stage.  And a pipeline whose
status is already defused (`… || true`) has said so on purpose.

THE FIX is to capture and then match without a pipe:

    out="$(some-command 2>&1)"
    if ! nros_grep_q 'needle' <<<"$out"; then …

A here-string has no writer PROCESS — bash fills the descriptor before grep is
started — so the race cannot exist.  Measured 0 of 5000.

The ALLOWLIST is a RATCHET, not an exoneration: those sites predate the gate
and sit outside the `check-fast` lane where this was measured.  Each is the
same shape and can bite the same way; convert one and delete its line.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

# Sites that predate this gate. Same shape, different lane. Delete a line once
# the site stops piping into its matcher.
#
# Keyed by the LINE TEXT, not by a line number: several of these files are
# edited by whoever is passing through, and a line-numbered ratchet turns any
# insertion above a listed site into a red that names the wrong thing. The cost
# is that two identical lines in one file share an entry, which is the right
# trade for a gate on the merge-queue path.
ALLOWLIST: dict[str, tuple[str, ...]] = {
    "docker/can-demo/run.sh": (
        "if ! modinfo vcan >/dev/null 2>&1 && ! lsmod 2>/dev/null | grep -q '^vcan'; then",
    ),
    "just/check.just": (
        'if rustup target list --installed | grep -qx aarch64-unknown-none; then',
    ),
    "just/workspace.just": (
        'if ! echo "$list_out" | grep -q "^$channel"; then',
        'if rustc +{{NIGHTLY}} --print target-list 2>/dev/null | grep -q armv7a-nuttx-eabi; then',
    ),
    "justfile": (
        'if [ -x "$bin" ] && "$bin" --version 2>/dev/null | grep -q "$want"; then',
        'if [ -x "$pinned_cf" ] && "$pinned_cf" --version 2>/dev/null | grep -q "$want_cf"; then',
    ),
    "packages/rmw/cyclonedds/nros-rmw-cyclonedds/tests/alloc_free_audit.sh": (
        'if ! rustup target list --installed 2>/dev/null | grep -qx "$TARGET"; then',
    ),
    "scripts/build-all-jobserver.sh": (
        'if [ ! -x "$make_bin" ] || ! "$make_bin" --version | head -1 | grep -q "4.4"; then',
    ),
    "scripts/build/compile-check-fixtures.sh": (
        'if ! rustup target list --installed 2>/dev/null | grep -qx "$target"; then',
        'if find "$staged" -maxdepth 3 -name package.xml -print -quit 2>/dev/null | grep -q .; then',
    ),
    "scripts/build/fixture-make-driver.sh": (
        '"$(nros sdk-path make)/bin/make" --version | head -1 | grep -q "4.4" && \\',
    ),
    "scripts/build/fixtures-build.sh": (
        '"$(nros sdk-path make)/bin/make" --version | head -1 | grep -q "4.4"; then',
    ),
    "scripts/build/workspace-fixtures-build.sh": (
        '[ -x "$pinned_make" ] && "$pinned_make" --version | head -1 | grep -q "4.4"; then',
    ),
    "scripts/build/zephyr-fixture-leaves.sh": (
        'if [ "$rmw" = "cyclonedds" ] && ! printf \'%s\\n\' "${fixture_rmws[@]}" | grep -qx cyclonedds; then',
    ),
    "scripts/build/zephyr-fixture-make-driver.sh": (
        '"$(nros sdk-path make)/bin/make" --version | head -1 | grep -q "4.4"; then',
    ),
    "scripts/check-artifact-identity-budget.sh": (
        '&& ! printf \'%s\\n\' "$rlibs" | grep -q "/lib${BUDGET_CRATE}-" \\',
        '&& printf \'%s\\n\' "$_all" | grep -q "/lib${BUDGET_CRATE}-"; then',
    ),
    "scripts/check-sched-dim-arms-compile.sh": (
        'elif ! printf \'%s\' "$out" | grep -E \'error|note\' | grep -q \'threadx_hooks[.]c\'; then',
        'elif printf \'%s\' "$out" | grep -qE \'is required in SMP|must also be defined|Missing definition\'; then',
    ),
    "scripts/check-tier-preconditions.sh": (
        '! "$_pre_make" --version 2>/dev/null | head -1 | grep -q "4\\.4"; then',
    ),
    "scripts/cyclonedds/ddsrt-port-inventory.sh": (
        'if find "$ddsrt_dir" -path \'*threadx*\' -print -quit | grep -q .; then',
    ),
    "scripts/dev/sweep-vendored-c-decls.sh": (
        'elif printf \'%s\' "$err" | grep -qE \'fatal error:.*No such file\'; then',
        'if printf \'%s\' "$err" | grep -qE \'error:.*(implicit declaration|makes pointer from integer|int-conversion)\'; then',
    ),
    "scripts/reserve-issue-id.sh": (
        'if printf \'%s\' "$err" | grep -qiE "already exists|non-fast-forward|fetch first|rejected"; then',
    ),
    "scripts/reserve-phase-id.sh": (
        'if printf \'%s\' "$err" | grep -qiE "already exists|non-fast-forward|fetch first|rejected"; then',
    ),
    "scripts/ros/domain-bridge-repro.sh": (
        'if ! echo "$OUT" | grep -qF "$e"; then',
        'if [ -z "$OUT" ] || echo "$OUT" | grep -qiE "no message|timed out"; then',
    ),
    "scripts/zephyr/cargo-features-patch.sh": (
        '"$_d/CMakeLists.txt" | grep -q .; then',
    ),
    "tools/setup.sh": (
        '&& ! list_known_platforms | grep -qx "$PLATFORM" \\',
    ),
}

# A last stage that stops reading before EOF: a MATCHER PREDICATE, whose only
# product is an exit status, so it stops the moment it knows the answer.
EARLY_EXIT_PREDICATE = re.compile(
    r"^\s*(?:"
    r"nros_grep_q\b"                                # the sanctioned helper,
    r"|nros_git_grep_q\b"                           # piped into anyway
    r"|(?:command\s+)?(?:git\s+)?grep\s+(?:"
    r"-[A-Za-z]*q[A-Za-z]*\b|--quiet\b"             # grep -q / -qE / -qF / -qx
    r"|-[A-Za-z]*m[A-Za-z]*\s*\d|--max-count"       # grep -m N
    r")"
    r")"
)

# The pipeline's exit status is READ rather than discarded. A bare `||` is
# absent on purpose: `… || true` / `… || :` is the status being defused
# deliberately, and `… || continue` reaches us through its own arm.
STATUS_CONSUMED = (
    re.compile(r"^\s*(?:if|elif|while|until)\s"),
    re.compile(r"^\s*!\s"),
    re.compile(r";\s*then\s*$"),
    re.compile(r"\|\|\s*(?:continue|break|return|exit|fail\b)"),
    re.compile(r"&&\s*\S"),
    re.compile(r"\\\s*$"),          # continued into an && / ; then
)


def last_pipeline_stage(line: str) -> str:
    """The text after the final single `|`, ignoring `||` and quoted text.

    Crude on purpose: this reads one LINE of shell, not a parse tree. It is
    enough to tell `a | head -1 | sed …` (a value) from `a | head -1 | grep -q X`
    (a predicate), and to keep a `|` inside a pattern — `grep -qE 'a|b'` — from
    reading as a pipe. Those two are the distinctions that decide whether a
    SIGPIPE on the writer can be mistaken for a verdict.
    """
    out: list[str] = []
    i = 0
    quote = ""          # "'" or '"' while inside a quoted run
    while i < len(line):
        c = line[i]
        if quote:
            # Inside single quotes bash has no escapes at all; inside double
            # quotes a backslash escapes the next byte.
            if quote == '"' and c == "\\" and i + 1 < len(line):
                out.append(c)
                out.append(line[i + 1])
                i += 2
                continue
            if c == quote:
                quote = ""
            out.append(c)
            i += 1
            continue
        if c == "\\" and i + 1 < len(line):
            out.append(c)
            out.append(line[i + 1])
            i += 2
            continue
        if c in ("'", '"'):
            quote = c
            out.append(c)
            i += 1
            continue
        if c == "|":
            if i + 1 < len(line) and line[i + 1] == "|":
                out.append("||")
                i += 2
                continue
            out.append("\x00")
            i += 1
            continue
        out.append(c)
        i += 1
    joined = "".join(out)
    if "\x00" not in joined:
        return ""
    return joined.rsplit("\x00", 1)[1]


def flags(line: str) -> bool:
    """Does this ONE line carry the defect?"""
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return False
    stage = last_pipeline_stage(line)
    if not stage or not EARLY_EXIT_PREDICATE.match(stage):
        return False
    return any(rx.search(line) for rx in STATUS_CONSUMED)


# The detector is a regex over one line of shell, so it can stop matching
# without anyone noticing and report a clean sweep over a class it no longer
# sees — issue 0196's rule failing in miniature. These hold it to BOTH
# directions on the exact shapes that motivated it.
SELF_TEST = (
    # (line, expected-to-be-flagged, why)
    ("""if ! printf '%s' "$_o_out" | grep -q "status=refused"; then""", True,
     "the literal line from issue 1077"),
    ("""elif printf '%s' "$out" | grep -qE 'is required|must also be'; then""", True,
     "an alternation `|` inside the PATTERN is not a pipe"),
    ("""    if ! "$m" --version | head -1 | grep -q "4.4"; then""", True,
     "an early-exit predicate behind an early-exit `head`"),
    ("""    printf '%s' "$b" | grep -qE 'add_executable\\(' || continue""", True,
     "`|| continue` reads the status just as `if` does"),
    ("""if ! printf '%s' "$out" | nros_grep_q 'needle'; then""", True,
     "the 0726 helper does not survive being piped into either"),
    ("""x="$(grep -E '^t' "$f" | head -1 | sed 's/a/b/')" """, False,
     "value position: the status is discarded and the value is unaffected"),
    ("""ver="$(qemu --version | head -1)" """, False,
     "value position with an early-exit last stage: still only a value"),
    ("""    variant="$(grep -oE 'x' "$h" | head -1 || true)" """, False,
     "`|| true` is the status defused on purpose"),
    ("""if ! nros_grep_q "status=refused" <<<"$_o_out"; then""", False,
     "the FIX must not trip the gate that asked for it"),
)


def self_test() -> list[str]:
    problems: list[str] = []
    for line, expected, why in SELF_TEST:
        if flags(line) != expected:
            problems.append(
                f"expected {'a HIT' if expected else 'no hit'} ({why}) on: {line.strip()}"
            )
    return problems


def tracked_shell() -> list[str]:
    """Tracked shell we author: `*.sh`, plus the `just` recipe bodies, which
    are bash too and run in the same fan-out."""
    out = subprocess.run(
        ["git", "ls-files", "*.sh", "*.just", "justfile"],
        capture_output=True, text=True, check=True,
    ).stdout.split()
    return sorted(
        f for f in out
        if "/third-party/" not in f and "/generated/" not in f
    )


def scan(root: Path) -> tuple[list[str], list[str]]:
    """Return (violations, stale allowlist entries)."""
    me = Path(__file__).resolve()
    violations: list[str] = []
    seen: set[tuple[str, str]] = set()
    for rel in tracked_shell():
        path = root / rel
        if path.resolve() == me:
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        # Only a file that turns on pipefail can suffer this: without it the
        # pipeline reports the matcher's status alone, which is the right
        # answer whatever the writer did.
        if "pipefail" not in text:
            continue
        allowed = ALLOWLIST.get(rel, ())
        for lineno, line in enumerate(text.splitlines(), 1):
            if not flags(line):
                continue
            stripped = line.strip()
            if stripped in allowed:
                seen.add((rel, stripped))
                continue
            violations.append(f"{rel}:{lineno}: {stripped[:110]}")
    stale = sorted(
        f"{rel}: {line}"
        for rel, lines in ALLOWLIST.items()
        for line in lines
        if (rel, line) not in seen
    )
    return violations, stale


def main() -> int:
    problems = self_test()
    if problems:
        print(
            "check-pipefail-sigpipe-assertions: SELF-TEST FAILED — the detector "
            "no longer recognises the shape it exists to find, so a green here "
            "would mean nothing.",
            file=sys.stderr,
        )
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1

    root = Path(__file__).resolve().parent.parent
    violations, stale = scan(root)

    if stale:
        print(
            "check-pipefail-sigpipe-assertions: the allowlist names "
            f"{len(stale)} line(s) that no longer match.",
            file=sys.stderr,
        )
        print(
            "  A stale entry silences a line that has MOVED, so delete it "
            "(or re-point it) rather than leaving it:",
            file=sys.stderr,
        )
        for site in stale:
            print(f"    {site}", file=sys.stderr)
        return 1

    if violations:
        print(
            "check-pipefail-sigpipe-assertions: FAILED (issue 1077) — "
            f"{len(violations)} pipeline(s) whose status is read can report a "
            "MATCH as a MISS.",
            file=sys.stderr,
        )
        print(file=sys.stderr)
        for v in violations:
            print(f"  {v}", file=sys.stderr)
        print(
            "\n"
            "  Under `set -o pipefail` a matcher that exits early (grep -q,\n"
            "  grep -m, nros_grep_q) can kill the writer with SIGPIPE, and 141\n"
            "  then becomes the pipeline's status — so a SUCCESSFUL match reads\n"
            "  as a failure, intermittently, under load.\n"
            "\n"
            "  Capture first, then match with a HERE-STRING:\n"
            "\n"
            "      out=\"$(some-command 2>&1)\"\n"
            "      if ! nros_grep_q 'needle' <<<\"$out\"; then ...\n"
            "\n"
            "  `nros_grep_q` is scripts/lib/grep-q.sh (issue 0726). Piping INTO\n"
            "  it does not help: the helper is then a subshell, so its own\n"
            "  fatal path cannot stop the caller either.",
            file=sys.stderr,
        )
        return 1

    print(
        "check-pipefail-sigpipe-assertions: OK "
        "(no unlisted status-consuming pipeline into an early-exit matcher; "
        f"{sum(len(v) for v in ALLOWLIST.values())} pre-existing "
        f"site(s) in {len(ALLOWLIST)} file(s) allowlisted)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
