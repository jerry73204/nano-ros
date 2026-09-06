#!/usr/bin/env python3
"""phase-348 W2 — discovery and resolution claim the same names.

A provider says what it IS twice: its `package.xml` announces
`<nano_ros_provides kind=… name=…/>` so a source-time scan can FIND it, and its
descriptor (`nros-rmw.toml`, `nros-board.toml`) declares the names a consumer
RESOLVES against. Nothing structural keeps the two equal, so this compares
them.

  A1  a package.xml sitting beside a descriptor announces provisions of that
      kind — otherwise it is invisible to the scan while looking migrated;
  A2  for a NAMED family, its provision names equal the descriptor's declared
      names EXACTLY and in order, canonical first, since names[0] is what
      error messages list;
  A2n for a NAMELESS family, the descriptor declares no names at all — the
      announcement is the only spelling (RFC-0087 D4).

**A2 and A2n are the same rule seen from two sides.** A family is nameless
once its readers DERIVE the names from the announcement instead of reading
them out of the descriptor; then a `names` key in a descriptor is not a
duplicate to be compared, it is a duplicate that nothing reads — worse than a
disagreement, because it can be edited with no effect. A2n refuses it, so the
rule "the announcement is the only spelling" keeps a gate after the comparison
it used to have stops existing.

**One gate for every family, not one per family.** This started as S5 inside
`check-rmw-descriptors.py`, covering rmw alone. Extending the rule to boards by
adding a second copy next to the board descriptors is exactly the
second-spelling antipattern this repo keeps paying for (see the Zephyr
unset-variable guard, #282 → #326), so S5 moved here instead and
`check-rmw-descriptors.py` kept only what is rmw-SPECIFIC (S1–S4). Adding a
`platform` family later means one row in FAMILIES, not another script.

A package.xml with no descriptor beside it is NOT checked: the migration
proceeds one provider at a time, and an unmigrated provider is simply not
discoverable while its existing build path keeps working.

Buildless — TOML plus a regex, no cmake, no cargo.
"""

import glob
import os
import re
import sys

try:
    import tomllib  # 3.11+
except ModuleNotFoundError:  # 3.10 backport, same spelling as the sibling gates
    import tomli as tomllib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# kind -> (descriptor glob, how to pull the declared names out of it).
#
# `extract=None` means a NAMELESS family: its readers derive the names from the
# `<nano_ros_provides>` announcements, so the descriptor must not declare any
# (A2n). Everything else about the row is unchanged — A1 still applies, because
# a provider that announces nothing is invisible to the scan whatever its
# descriptor says.
#
# The named families disagree in shape and that is not an accident: an rmw
# descriptor is ONE backend (`[rmw]`), while a board descriptor is an ARRAY
# (`[[board]]`) because one package can ship several boards — nros-board-nuttx-qemu
# declares both the ARM and RISC-V variants, disambiguated by `target_contains`.
# So the board reader flattens entries in declaration order.
FAMILIES = {
    # phase-420 W5 — rmw went NAMELESS. `cargo-nano-ros/build.rs` reads the
    # announcements (via `derived_descriptor::announced_names`) and derives the
    # rest of the lowering from names[0], so `[rmw].names` is deleted from all
    # four descriptors. A2 has nothing left to compare; A2n keeps it that way.
    "rmw": ("packages/rmw/*/*/nros-rmw.toml", None),
    # Board is still NAMED, and phase-420 W5 measured why rather than assuming:
    # `nros-board-nuttx-qemu` declares TWO `[[board]]` entries and announces
    # seven names in one flat list, so the announcement cannot say which four
    # belong to the ARM variant and which three to the RISC-V one. Deriving
    # per-entry names needs a boundary the tag has no way to carry. The board
    # reader also lives in `nros-cli-core`, not here.
    # phase-375 W6 / RFC-0064 R5 D2: TWO globs, because a bundle board lives one
    # level deeper (`nros-board-zephyr/boards/<bundle>/`). The Rust-side walk
    # became recursive; this gate does not walk, so it states both depths and
    # `check-board-glob-depth` holds them equal to what the walk finds.
    "board": (
        ["packages/boards/*/nros-board.toml", "packages/boards/*/boards/*/nros-board.toml"],
        lambda d: [n for b in d.get("board", []) for n in b.get("names", [])],
    ),
    # phase-349 W1. Platform descriptors live under `config/`, not
    # `packages/platform/` — a fact that cost a wrong "the family does not
    # exist" claim in phase-348 W2 (corrected there). `names` is top-level,
    # not in a table, and defaults to the directory name for a file that
    # declares none.
    "platform": (
        "config/*/nros-platform.toml",
        lambda d: list(d.get("names", [])),
    ),
    # phase-421 W4 / RFC-0088 D6 — the first family BORN nameless, where rmw was
    # made nameless afterwards by W5. `nros-serdes.toml` carries `impl` and
    # `format_id`, the two facts no convention can derive, and the announcement
    # is the name. Everything serdes-SPECIFIC — descriptor well-formedness, the
    # `format_id` discriminant, and the descriptor a package announcing `serdes`
    # must have — lives in `scripts/check-serdes-descriptors.py`, the split that
    # keeps this gate one gate for every family rather than one per family.
    "serdes": ("packages/*/*/nros-serdes.toml", None),
}

PROVIDES_RE = r'<nano_ros_provides\s+kind="{kind}"\s+name="([^"]+)"\s*/?>'
COMMENT_RE = re.compile(r"<!--([^-]|-[^-])*-->")


def declared_provisions(path, kind):
    """Provision names of one kind, in file order, comments stripped.

    The strip is not optional (issue 0516): a provider's package.xml documents
    the provision tag in a comment, and a regex cannot tell that from a
    declaration. Without it a commented-out example counts as a claimed name.
    """
    with open(path, encoding="utf-8") as fh:
        body = COMMENT_RE.sub("", fh.read())
    return re.findall(PROVIDES_RE.format(kind=kind), body)


def restated_names(data):
    """Every `names` key anywhere in a descriptor, flattened.

    A nameless family's descriptor must declare none, and the search is over
    the WHOLE document rather than one known table: the two live shapes put
    `names` in `[rmw]`, in `[[board]]` entries and at top level
    (`nros-platform.toml`), and a rule that only looked where today's families
    happen to keep it would pass the next family's restatement.
    """
    out = []

    def walk(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k == "names" and isinstance(v, list):
                    out.extend(v)
                else:
                    walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)

    walk(data)
    return out


def scan(root):
    """(problems, checked, announced) for the tree at `root`.

    Split out of `main` so the negative control below can run the real rule
    against a fixture tree instead of asserting about it in prose. The gate had
    no control at all, which is how the "not migrated yet" skip below survived
    its own migration.
    """
    return _scan_impl(root)


def self_test(quiet=False):
    """The rule must FIRE on a descriptor with no package.xml.

    Runs on the NORMAL path, not behind a flag: a control nobody runs decays
    into a comment. Same shape as `scripts/check-board-tiers.py`.
    """
    import shutil
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        for kind, (pattern, _) in FAMILIES.items():
            pat = pattern if isinstance(pattern, str) else pattern[0]
            # Materialise one minimal descriptor per family so no family trips
            # the "refusing to pass on an empty set" arm.
            rel = pat.replace("*", "x")
            dest = os.path.join(tmp, rel)
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            body = 'names = ["x"]\n' if kind == "platform" else ""
            if kind == "board":
                body = '[[board]]\nnames = ["x"]\n'
            elif kind == "rmw":
                body = "[rmw]\n"
            with open(dest, "w", encoding="utf-8") as fh:
                fh.write(body)
            if kind != "board":
                # Every family EXCEPT the one under test gets its announcement,
                # so the assertion below names exactly one offender.
                names = ["x"] if kind == "platform" else []
                tags = "".join(
                    f'<nano_ros_provides kind="{kind}" name="{n}"/>' for n in names
                )
                with open(
                    os.path.join(os.path.dirname(dest), "package.xml"),
                    "w",
                    encoding="utf-8",
                ) as fh:
                    fh.write(f"<package><export>{tags}</export></package>")

        problems, _, _ = scan(tmp)
        offenders = [p for p in problems if "no sibling package.xml" in p]
        assert len(offenders) == 1, (
            "the rule must name the descriptor with no package.xml, and only "
            f"it; got {problems}"
        )

        # And the intended escape: announcing it silences the rule.
        board_pat = FAMILIES["board"][0]
        board_pat = board_pat if isinstance(board_pat, str) else board_pat[0]
        board_dir = os.path.dirname(os.path.join(tmp, board_pat.replace("*", "x")))
        with open(os.path.join(board_dir, "package.xml"), "w", encoding="utf-8") as fh:
            fh.write(
                '<package><export><nano_ros_provides kind="board" name="x"/>'
                "</export></package>"
            )
        problems, _, _ = scan(tmp)
        offenders = [p for p in problems if "no sibling package.xml" in p]
        assert not offenders, f"an announced descriptor must not be reported; got {problems}"
        shutil.rmtree(tmp, ignore_errors=True)

    if not quiet:
        print("check-provider-announcements self-test: OK")
    return 0


def _scan_impl(root):
    problems = []
    checked = 0
    announced = 0

    for kind, (pattern, extract) in sorted(FAMILIES.items()):
        # A family may state several globs (boards do: a bundle board sits one
        # level deeper). `dict.fromkeys` rather than `set`, so a path matched by
        # two globs is reported once and the order stays deterministic.
        patterns = [pattern] if isinstance(pattern, str) else list(pattern)
        paths = sorted(
            dict.fromkeys(
                m for pat in patterns for m in glob.glob(os.path.join(root, pat))
            )
        )
        if not paths:
            # A family whose descriptors all vanished would otherwise make this
            # gate quietly vacuous for that family.
            problems.append(
                f"family {kind!r}: no descriptor matched {patterns!r} — refusing to "
                f"pass on an empty set"
            )
            continue

        for desc_path in paths:
            desc_rel = os.path.relpath(desc_path, root)
            pkg_xml = os.path.join(os.path.dirname(desc_path), "package.xml")
            if not os.path.exists(pkg_xml):
                # RFC-0064 R5 D5 / phase-375 W6. This used to `continue`, with
                # the comment "not migrated yet; not discoverable, still
                # builds". The migration finished; the skip did not, and it
                # outlived its reason by long enough that phase-385 landed
                # `nros-board-mps3-an536-freertos` -- the NEWEST board in the
                # tree -- with a descriptor and no announcement, and no gate
                # said so. An optional step is a step the next person omits.
                problems.append(
                    f"{desc_rel}: descriptor with no sibling package.xml. A "
                    f"provider announces what it IS in package.xml and what it "
                    f"LOWERS TO in the descriptor; with only the second half it "
                    f"is invisible to `provider_scan` and to every consumer "
                    f"that reaches providers through it.\n"
                    f"    Add {os.path.relpath(pkg_xml, root)} with "
                    f'<nano_ros_provides kind="{kind}" .../> matching the '
                    f"descriptor's names, canonical first."
                )
                continue
            checked += 1
            rel = os.path.relpath(pkg_xml, root)

            with open(desc_path, "rb") as fh:
                try:
                    data = tomllib.load(fh)
                except Exception as e:  # noqa: BLE001 — report, do not raise
                    problems.append(f"{desc_rel}: not valid TOML: {e}")
                    continue
            names = None if extract is None else extract(data)
            if extract is None:
                # A2n — the announcement is the only spelling.
                restated = restated_names(data)
                if restated:
                    problems.append(
                        f"{desc_rel}: declares names {restated} in a NAMELESS "
                        f"family — {kind} names come from the "
                        f'<nano_ros_provides kind="{kind}"/> announcements and '
                        f"nothing reads this key, so it can drift with no "
                        f"symptom. Delete it (RFC-0087 D4)"
                    )
            elif not names:
                problems.append(
                    f"{desc_rel}: declares no names — nothing could resolve to it"
                )
                continue

            found = declared_provisions(pkg_xml, kind)
            announced += len(found)
            if not found:
                problems.append(
                    f'{rel}: sits beside a {kind} descriptor but announces no '
                    f'<nano_ros_provides kind="{kind}"/> — it would be invisible '
                    f"to the phase-348 scan"
                )
            elif names is not None and found != names:
                problems.append(
                    f"{rel}: provides {found} but {desc_rel} declares {names} — "
                    f"discovery and resolution must claim the same names, "
                    f"canonical first"
                )

    return problems, checked, announced


def main():
    rc = self_test(quiet=True)
    if rc:
        return rc

    problems, checked, announced = scan(ROOT)

    if problems:
        sys.stderr.write("check-provider-announcements: FAILED\n")
        for p in problems:
            sys.stderr.write(f"  {p}\n")
        return 1

    nameless = sorted(k for k, (_, e) in FAMILIES.items() if e is None)
    print(
        f"provider announcements: OK ({checked} migrated provider(s) across "
        f"{len(FAMILIES)} famil(ies), {announced} name(s) announced; "
        f"named families match their descriptor, nameless "
        f"({', '.join(nameless) or 'none'}) restate nothing)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
