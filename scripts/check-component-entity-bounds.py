#!/usr/bin/env python3
"""Issue 0857 — every `nros::node!` class must DECLARE its cell registry bounds.

WHAT THE CELL COSTS WHEN NOBODY DECLARES
----------------------------------------
`nros::node!(C)` emits `static __NROS_COMPONENT_<pkg>_SLOT_STORE:
ComponentSlotStorage<C, MAX_CLASS_INSTANCES, PUBS, SVCS, ACTC, ACTS, SSRV>`,
whose registry capacities come from `<C as Node>::ENTITY_BOUNDS`. That const
DEFAULTS to `EntityBounds::knob_caps()` — `NROS_RUNTIME_MAX_CELL_ENTITIES` (8)
per kind — and one publisher slot embeds a `TxArena<DEFAULT_LOAN_BUF>`, so the
default is not a small over-allocation. Measured with `nm -S` on
`examples/workspaces/rust`'s prebuilt native entries (2026-09-05):

    service_server_pkg  (no ENTITY_BOUNDS)      50,824 B   .bss
    action_server_pkg   (no ENTITY_BOUNDS)      50,832 B   .bss
    talker_pkg          (exact(1,0,0,0,0))       3,888 B   .bss
    listener_pkg        (exact(0,0,0,0,0))         272 B   .bss

Declaring is the whole difference, and nothing made a class declare — 81 of the
100 in-tree component classes did not. This gate is what stops the next one
from being added silently.

WHAT IT CHECKS, AND WHY ONLY THAT
---------------------------------
Two things, both one-directional:

1. **Presence.** A class the macro emits a store for states `ENTITY_BOUNDS`.
   No number is inferred; the author states it.

2. **A LOWER BOUND on the numbers.** `register()` bodies in this tree are
   straight-line `create_*` calls, so counting them gives a floor on what the
   cell registries will hold. A declaration BELOW that floor is a registration
   error waiting at boot (`push_publisher` returns `Err` → `NodeDeclError`), so
   it fails here instead.

This is a VERIFIER, never a supplier. It cannot prove a declaration is big
ENOUGH — a `for` loop could create more than the text shows — which is why the
runtime keeps its loud `Err`. It exists to catch the stale declaration in the
safe direction, the way `leaf_entity_env::reconcile` does one layer up for the
pool budgets (issues 0827/1061). The counting rules for BUDGETS live in
`nros_cli_core::entity_inventory` and are not duplicated here: this file counts
call sites in Rust source, which that module explicitly cannot do (it reads a
probe's JSON), and it counts them for a different question — the per-COMPONENT
per-KIND cell capacity, not the per-IMAGE pool demand.

SUBSCRIPTIONS AND TIMERS ARE NOT COUNTED, DELIBERATELY
------------------------------------------------------
The cell holds five registries: publishers, service clients, action clients,
action servers, and the service-server trampoline-context slab. A subscription
or a timer claims an EXECUTOR callback slot (`NROS_EXECUTOR_MAX_CBS`) and no
cell slot at all — the same split `entity_inventory::EntityKind::callback_slots`
spells for the other direction.

Usage:
    python3 scripts/check-component-entity-bounds.py [--self-test] [--report]
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

# Directories that hold Rust sources worth scanning. `third-party/` and build
# output are never ours; `generated/` is codegen'd per host and holds no
# components.
SCAN_ROOTS = ("packages", "examples")

SKIP_DIR_PARTS = (
    "/third-party/",
    "/generated/",
    "/target/",
    "/.git/",
    "/build/",
)

# A macro invocation at statement level. Inside a doc comment the line starts
# with `///` or `//!`, and inside a string template it is indented under a
# `quote!`/`format!` — neither of which this anchors on, so both are excluded by
# requiring the invocation to start the line (after indentation) and the file to
# also contain the matching `impl Node for`.
#
# The class name also matches a `{placeholder}` so the two SCAFFOLD templates
# — Rust source inside a `format!` string — are checked like any other class.
# They are the one place a missing declaration would propagate to every project
# a user generates, so they are the last place to leave unchecked.
NAME = r"(?:[A-Za-z_][A-Za-z0-9_]*|\{[A-Za-z_][A-Za-z0-9_]*\})"
NODE_MACRO_RE = re.compile(
    rf"^[ \t]*(?:::)?nros::node!\(\s*({NAME})\s*\)|^[ \t]*node!\(\s*({NAME})\s*\)",
    re.MULTILINE,
)

IMPL_NODE_RE = r"impl\s+Node\s+for\s+{name}\s*\{{"

# `EntityBounds::exact(p, ss, sc, ac, as)` — the five bounds, positionally, in
# the order the constructor takes them.
EXACT_RE = re.compile(
    r"EntityBounds::exact\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)"
)

# The five cell registries, and the `create_*` prefixes that fill each. Order
# matches `EntityBounds::exact`'s parameters so a mismatch cannot be silent.
#
# `create_service_static` / `create_action_static` are the tag-only SERVER
# spellings (they delegate to `create_service_server_for_name` /
# `create_action_server_for_name`), so they count as servers.
KINDS = (
    ("publishers", ("create_publisher",)),
    ("service_servers", ("create_service_server", "create_service_static")),
    ("service_clients", ("create_service_client",)),
    ("action_clients", ("create_action_client",)),
    ("action_servers", ("create_action_server", "create_action_static")),
)


def strip_comments(src: str) -> str:
    """Blank out line comments so a commented-out `create_*` is not counted.

    Line-wise and deliberately crude: a `//` inside a string literal would be
    blanked too. That direction is safe here — it can only LOWER the counted
    floor, never raise it above what the code creates.
    """
    out = []
    for line in src.splitlines():
        idx = line.find("//")
        out.append(line if idx < 0 else line[:idx])
    return "\n".join(out)


def block_from(src: str, start: int) -> str:
    """The `{ ... }` block whose opening brace is at or after `start`."""
    i = src.find("{", start)
    if i < 0:
        return ""
    depth = 0
    for j in range(i, len(src)):
        if src[j] == "{":
            depth += 1
        elif src[j] == "}":
            depth -= 1
            if depth == 0:
                return src[i : j + 1]
    return src[i:]


def counts_in(body: str) -> dict[str, int]:
    """Per-kind `create_*` call sites in one `register()` body."""
    out = {}
    for kind, prefixes in KINDS:
        n = 0
        for p in prefixes:
            n += len(re.findall(r"\." + p + r"[A-Za-z0-9_]*\s*(?:::<|\()", body))
        out[kind] = n
    return out


class Finding:
    def __init__(self, path: str, cls: str, kind: str, detail: str):
        self.path, self.cls, self.kind, self.detail = path, cls, kind, detail

    def __str__(self) -> str:
        return f"  {self.path}: {self.cls}: {self.detail}"


def scan_source(src: str, rel: str) -> tuple[list[Finding], list[tuple[str, dict, tuple | None]]]:
    """Findings plus `(class, counted, declared)` rows for the report."""
    findings: list[Finding] = []
    rows: list[tuple[str, dict, tuple | None]] = []
    clean = strip_comments(src)
    classes = []
    for m in NODE_MACRO_RE.finditer(clean):
        classes.append(m.group(1) or m.group(2))
    for cls in classes:
        im = re.search(IMPL_NODE_RE.format(name=re.escape(cls)), clean)
        if not im:
            # No `impl Node for <cls>` here: a doc example, a scaffold template
            # string, or an impl in another file. Nothing to check, and nothing
            # to claim — the macro is only load-bearing where the impl is.
            continue
        # From the END of the match, which IS the opening brace: starting at
        # `im.start()` finds the first `{` after `impl`, and for a scaffold
        # template's `impl Node for {type_name}` that is the PLACEHOLDER, so
        # the "body" came back as `{type_name}` and every check read empty.
        impl_body = block_from(clean, im.end() - 1)
        reg = re.search(r"fn\s+register\s*\(", impl_body)
        counted = counts_in(block_from(impl_body, reg.start()) if reg else "")
        dm = EXACT_RE.search(impl_body)
        declared = tuple(int(g) for g in dm.groups()) if dm else None
        rows.append((cls, counted, declared))
        if "ENTITY_BOUNDS" not in impl_body:
            findings.append(
                Finding(
                    rel,
                    cls,
                    "absent",
                    "states no `ENTITY_BOUNDS`, so its static cell is sized at "
                    "`NROS_RUNTIME_MAX_CELL_ENTITIES` per kind "
                    f"(measured 50,824 B of .bss for this shape). Declare "
                    f"`const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact"
                    f"({', '.join(str(counted[k]) for k, _ in KINDS)});`",
                )
            )
            continue
        if declared is None:
            # `knob_caps()` or a spelling this gate cannot read. Explicit is
            # fine — the author stated the worst case on purpose.
            continue
        for i, (kind, _) in enumerate(KINDS):
            if declared[i] < counted[kind]:
                findings.append(
                    Finding(
                        rel,
                        cls,
                        "short",
                        f"declares {kind}={declared[i]} but `register()` creates "
                        f"{counted[kind]}. A short registry is a registration "
                        f"error at boot, not a saving.",
                    )
                )
    return findings, rows


def rust_files() -> list[Path]:
    """Every TRACKED `.rs` under the scan roots.

    `git ls-files`, not a walk: `check-no-tracked-file-find` measured the same
    question at 7m36s by `find` and 0.8s by the index. It also answers the
    right question — a component class lives in a tracked file, and an
    untracked `generated/` or `target/` tree holds none.
    """
    out = subprocess.run(
        ["git", "ls-files", "-z", "--", *[f"{r}/**/*.rs" for r in SCAN_ROOTS]],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    paths = []
    for rel in out.split("\0"):
        if not rel:
            continue
        if any(part in "/" + rel for part in SKIP_DIR_PARTS):
            continue
        paths.append(REPO / rel)
    return sorted(paths)


SELF_TESTS = [
    (
        "absent bounds is a finding",
        """
impl Node for Talker {
    const NAME: &'static str = "talker";
    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let p = node.create_publisher_for_topic::<Int32>("/chatter")?;
        Ok(())
    }
}
nros::node!(Talker);
""",
        1,
    ),
    (
        "exact bounds matching the body passes",
        """
impl Node for Talker {
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);
    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let p = node.create_publisher_for_topic::<Int32>("/chatter")?;
        Ok(())
    }
}
nros::node!(Talker);
""",
        0,
    ),
    (
        "a declaration SHORTER than the body is a finding",
        """
impl Node for Server {
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);
    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let s = node.create_service_server_for_name::<AddTwoInts>("/add")?;
        Ok(())
    }
}
nros::node!(Server);
""",
        1,
    ),
    (
        "the tag-only static spellings count as servers",
        """
impl Node for Server {
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);
    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let s = node.create_service_static::<AddTwoInts>("/add")?;
        let a = node.create_action_static::<Fib>("/fib")?;
        Ok(())
    }
}
nros::node!(Server);
""",
        2,
    ),
    (
        "subscriptions and timers claim NO cell slot",
        """
impl Node for Listener {
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);
    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        node.create_subscription_for_callback_name::<Int32>("/chatter", "on_msg")?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(10))?;
        Ok(())
    }
}
nros::node!(Listener);
""",
        0,
    ),
    (
        "a macro with no impl in the file is not this file's business",
        """
// A scaffold template, quoted:
let t = "nros::node!(Talker);";
""",
        0,
    ),
    (
        "a commented-out creator is not counted",
        """
impl Node for Talker {
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);
    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        // node.create_publisher_for_topic::<Int32>("/chatter")?;
        Ok(())
    }
}
nros::node!(Talker);
""",
        0,
    ),
    (
        "a scaffold template's placeholder class is checked, doubled braces and all",
        """
impl Node for {type_name} {{
    const NAME: &'static str = "{node_name}";
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);
    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {{
        let _t = node.create_timer_for_callback_name("on_tick", d)?;
        Ok(())
    }}
}}
nros::node!({type_name});
""",
        0,
    ),
    (
        "and a template that declares nothing is still a finding",
        """
impl Node for {type_name} {{
    const NAME: &'static str = "{node_name}";
    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {{
        let _p = node.create_publisher_for_topic::<M>("/t")?;
        Ok(())
    }}
}}
nros::node!({type_name});
""",
        1,
    ),
    (
        "an explicit knob_caps() declaration is a choice, not a finding",
        """
impl Node for Big {
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::knob_caps();
    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        for t in TOPICS { node.create_publisher_for_topic::<Int32>(t)?; }
        Ok(())
    }
}
nros::node!(Big);
""",
        0,
    ),
]


def self_test(quiet: bool = False) -> int:
    bad = 0
    for name, src, want in SELF_TESTS:
        got, _ = scan_source(src, "<self-test>")
        if len(got) != want:
            bad += 1
            print(f"  FAIL {name}: expected {want} finding(s), got {len(got)}")
            for f in got:
                print(f"    {f.detail}")
    if bad:
        print(f"\ncheck-component-entity-bounds --self-test: {bad} case(s) FAILED")
        return 1
    if not quiet:
        print(f"check-component-entity-bounds --self-test: {len(SELF_TESTS)} case(s) OK")
    return 0


def main() -> int:
    if "--self-test" in sys.argv:
        return self_test()
    # Always, not only behind the flag: a negative control nobody runs decays
    # into a comment, and this rule counts call sites with a regex — the one
    # kind of rule that silently stops firing.
    if self_test(quiet=True) != 0:
        return 1
    report = "--report" in sys.argv
    findings: list[Finding] = []
    scanned = 0
    classes = 0
    for path in rust_files():
        try:
            src = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if "node!(" not in src:
            continue
        scanned += 1
        rel = str(path.relative_to(REPO))
        f, rows = scan_source(src, rel)
        classes += len(rows)
        findings.extend(f)
        if report:
            for cls, counted, declared in rows:
                want = ", ".join(str(counted[k]) for k, _ in KINDS)
                have = "absent" if declared is None else str(declared)
                print(f"{rel}\t{cls}\texact({want})\t{have}")
    if findings:
        print("check-component-entity-bounds: FAIL\n")
        for f in findings:
            print(str(f))
        print(
            "\nEach class the `nros::node!` macro emits a store for pays its "
            "declared capacity in .bss,\ntwice over (`NROS_RUNTIME_MAX_CLASS_"
            "INSTANCES`). Undeclared means the 8-per-kind knob cap,\nwhich "
            "measured 50,824 B for a one-service component (issue 0857)."
        )
        return 1
    print(
        f"check-component-entity-bounds: OK ({classes} component class(es) "
        f"in {scanned} file(s))"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
