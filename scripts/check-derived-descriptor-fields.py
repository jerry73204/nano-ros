#!/usr/bin/env python3
"""RFC-0087 D4 / phase-420 W5 — a stated derivable field equals its derived value.

A provider descriptor carries only what no convention can produce. Measured on
`nros-rmw-zenoh/nros-rmw.toml` on 2026-09-04, six of eleven fields were
convention written longhand:

    names          the `<nano_ros_provides>` announcements, restated
    cargo_feature  `<kind>-<name>`
    cmake_value    the canonical name
    c_define_token `UPPER(name)`
    cffi_feature   `<cargo_feature>-cffi`
    crate          the package's own `Cargo.toml`

W5 deleted the first five from the rmw descriptors and made
`cargo-nano-ros/src/derived_descriptor.rs` produce them. This gate is what
keeps them from coming back WRONG: a descriptor may still state one — an
out-of-tree provider written against the old shape, or a family whose reader
has not moved yet — and then it must equal what the convention produces.

## The rules

  D1  a stated derivable field equals its derived value;
  D2  a stated field whose derivation is UNAVAILABLE here (no `package.xml` to
      announce, no `Cargo.toml` to name a crate) is a claim nothing can check,
      and must be grandfathered rather than assumed right;
  D3  the baseline only shrinks: a row whose divergence is gone is REPORTED as
      improved, and a row naming a descriptor that no longer exists is an
      error, because a stale exemption is how a rule gets reclaimed by
      accident;
  D4  no family is vacuous — a glob that matches nothing, or a scan that
      checked no field at all, fails instead of printing OK.

## Ratchet

`scripts/derived-descriptor-fields-baseline.json`, the shape and the reasoning
of `scripts/board-maintainer-baseline.json` and
`scripts/build-type-spelling-baseline.json`: history is grandfathered where a
spelling cannot change, and new drift is refused. The VALUE stored is the
spelling that was grandfathered, so swapping one divergence for a different
one is a NEW violation rather than a covered one.

Improving a row is REPORTED, never failed — a ratchet that punishes the good
deed gets bypassed, which is `build-type-spelling`'s rule and this one's too.

## What is NOT here

**`names` is checked by `check-provider-announcements`, not by this gate.**
That gate already compares a named family's `names` against the announcements
(A2) and refuses a nameless family's restatement outright (A2n) — which is
exactly "the stated value equals the derived value" for that one field, from
the other side. A copy of the rule here would be the second spelling this
whole wave exists to delete.

**`cpp_define` is not derivable and is not checked.** Its own descriptor
comment says the spellings "are INCONSISTENT across backends by history
(`_CFFI` on two, bare on two) and are preserved exactly — consumers `#if` on
them". A convention cannot produce it, so it stays authored.

Dependency-light: TOML via the 3.10 `tomli` backport, the same spelling as the
sibling gates; no cmake, no cargo, no build.
"""

import json
import re
import sys
import tempfile
from pathlib import Path

try:
    import tomllib  # 3.11+
except ModuleNotFoundError:  # 3.10 backport, same spelling as the sibling gates
    import tomli as tomllib

ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / "scripts/derived-descriptor-fields-baseline.json"

PROVIDES_RE = r'<nano_ros_provides\s+kind="{kind}"\s+name="([^"]+)"\s*/?>'
# An XML comment body cannot contain `--`, so this matches one exactly. The
# strip is not optional (issue 0516): every provider package.xml documents the
# announcement tag in a comment above the real one, and a regex cannot tell a
# doc example from a claim.
COMMENT_RE = re.compile(r"<!--([^-]|-[^-])*-->")

# A sentinel distinct from None (= "not derivable here", D2) and from "" (= a
# legitimately empty derived value).
UNAVAILABLE = object()


# ---------------------------------------------------------------------------
# The conventions. One Python copy of `cargo-nano-ros/src/derived_descriptor.rs`
# — unavoidable, because this gate is buildless and that one is Rust. It is a
# CHECKER of the rule, not a second producer: nothing consumes what it computes,
# and `rmw_resolver::the_scalar_lowerings_are_the_convention` asserts the Rust
# side against the shared implementation, so the two cannot both be wrong
# silently.
# ---------------------------------------------------------------------------
def conv_cargo_feature(kind, name):
    return f"{kind}-{name}"


def conv_cmake_value(_kind, name):
    return name


def conv_c_define_token(_kind, name):
    return "".join(c.upper() if c.isascii() and c.isalnum() else "_" for c in name)


def conv_cffi_feature(kind, name):
    return conv_cargo_feature(kind, name) + "-cffi"


def announced_names(pkg_dir, kind):
    """The names a package's `package.xml` announces, in file order."""
    pkg_xml = pkg_dir / "package.xml"
    if not pkg_xml.is_file():
        return None
    body = COMMENT_RE.sub("", pkg_xml.read_text(encoding="utf-8"))
    return re.findall(PROVIDES_RE.format(kind=kind), body)


def own_crate_name(pkg_dir):
    """`[package] name` of the `Cargo.toml` beside the descriptor, if any."""
    manifest = pkg_dir / "Cargo.toml"
    if not manifest.is_file():
        return None
    try:
        data = tomllib.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    name = data.get("package", {}).get("name")
    return name if isinstance(name, str) else None


# RFC-0064 R5 D6 / phase-375 W8 — derivations whose input is a SIBLING FIELD in
# the same entry, rather than the canonical name or the package directory.
#
# Boards need this shape and the other two families do not: a board's
# `crate_name` is a transform of its `board_crate` one line above it, and its
# `local_aliases` default is its own `platform_feature`. Returning `None` means
# the input is absent, which is D2's "unavailable" rather than a divergence.


def entry_crate_name(entry):
    """`snake_case(board_crate)` — 7 of 7 boards that state one state this."""
    bc = entry.get("board_crate")
    return bc.replace("-", "_") if isinstance(bc, str) else None


# phase-375 W8 — the PLATFORM's facts, stated on a board.
#
# `platform_feature` was stated by all 15 rows and 3 of them stated something
# that is not `platform-<platform>`: esp32 selects `platform-bare-metal`, and
# BOTH ThreadX kinds select `platform-threadx`. That is the rule, not three
# exceptions, and it is a property of the platform being copied onto every
# board that uses it — where it can be got wrong.
_PLATFORM_FEATURE = {
    "posix": "platform-posix",
    "freertos": "platform-freertos",
    "bare-metal": "platform-bare-metal",
    "esp32": "platform-bare-metal",
    "nuttx": "platform-nuttx",
    "zephyr": "platform-zephyr",
    "threadx-linux": "platform-threadx",
    "threadx-riscv64": "platform-threadx",
    "stm32": "platform-bare-metal",
    "orin-spe": "platform-bare-metal",
}

# 13 of 15 said `none`; the two that said `nuttx-staging` are the two NuttX
# rows. A platform fact stated fifteen times.
_LINK_KIND = {"nuttx": "nuttx-staging"}


def entry_platform_feature(entry):
    plat = entry.get("platform")
    return _PLATFORM_FEATURE.get(plat) if isinstance(plat, str) else None


def entry_link_kind(entry):
    plat = entry.get("platform")
    return _LINK_KIND.get(plat, "none") if isinstance(plat, str) else None


def entry_local_aliases(entry):
    """`[platform_feature]` — 8 of 10 stated exactly that.

    The two real overrides (`platform-esp32-qemu`, `platform-threadx-riscv64`)
    are DIVERGENCES by this rule and are grandfathered in the baseline, which is
    the correct outcome: they are deliberate, they are few, and a reader who
    meets one should be able to see that it was noticed.
    """
    pf = entry.get("platform_feature")
    return [pf] if isinstance(pf, str) else None


ENTRY_DERIVED = {
    "crate_name": entry_crate_name,
    "local_aliases": entry_local_aliases,
    "platform_feature": entry_platform_feature,
    "link_kind": entry_link_kind,
}

# ---------------------------------------------------------------------------
# Families. A row says where the descriptors are, how to walk their entries,
# and which fields in each entry are convention.
#
# `names` is deliberately absent from every field map — see the module header.
# ---------------------------------------------------------------------------
def rmw_entries(data):
    """One backend per descriptor: `[rmw]`, plus `[rmw.provides.cargo]`."""
    rmw = data.get("rmw")
    if not isinstance(rmw, dict):
        return []
    provides = rmw.get("provides", {})
    cargo = provides.get("cargo", {}) if isinstance(provides, dict) else {}
    merged = {k: v for k, v in rmw.items() if not isinstance(v, dict)}
    if isinstance(cargo, dict) and "crate" in cargo:
        merged["crate"] = cargo["crate"]
    return [("rmw", merged)]


def board_entries(data):
    """An ARRAY of boards: one package can ship several (nros-board-nuttx-qemu
    declares the ARM and RISC-V variants), so entries are labelled by index."""
    boards = data.get("board")
    if not isinstance(boards, list):
        return []
    out = []
    for i, b in enumerate(boards):
        if not isinstance(b, dict):
            continue
        # `[board.entry]`'s fields are flattened in beside the board's own, so a
        # sibling-field derivation (`crate_name` from `board_crate`) can see
        # both. Flattening is safe because the two key sets are disjoint; a
        # collision would silently pick one, so it is asserted.
        entry = b.get("entry")
        merged = {k: v for k, v in b.items() if k != "entry"}
        if isinstance(entry, dict):
            assert not (set(entry) & set(merged)), (
                f"board[{i}]: `[board.entry]` and the board share a key "
                f"({sorted(set(entry) & set(merged))}); flattening would hide one"
            )
            merged.update(entry)
        out.append((f"board[{i}]", merged))
    return out


FAMILIES = {
    "rmw": {
        "glob": "packages/rmw/*/*/nros-rmw.toml",
        "entries": rmw_entries,
        # field -> (canonical-name source, derivation)
        "fields": {
            "cargo_feature": conv_cargo_feature,
            "cmake_value": conv_cmake_value,
            "c_define_token": conv_c_define_token,
            "cffi_feature": conv_cffi_feature,
            # RFC-0087 D4 lists `crate` as derivable "from the package's own
            # Cargo.toml". phase-420 W5 measured that and it is NOT true here:
            # `nros-rmw-xrce` ships no Cargo.toml and names a SIBLING package.
            # It is checked, and the exception is grandfathered rather than
            # explained away.
            "crate": None,
        },
    },
    "board": {
        # TWO globs since phase-375 W6: a bundle board lives one level deeper
        # (`nros-board-<family>/boards/<bundle>/`). A single-glob version of
        # this list is how `fvp-aemv8r-smp` went unchecked by several gates at
        # once.
        "glob": [
            "packages/boards/*/nros-board.toml",
            "packages/boards/*/boards/*/nros-board.toml",
        ],
        "entries": board_entries,
        "fields": {
            # RFC-0064 R5 D6.
            "local_aliases": None,
            "crate_name": None,
            # phase-375 W8 — the platform's, not the board's.
            "platform_feature": None,
            "link_kind": None,
            # The board's own crate. Every board that states one states its own
            # package name; the three crate-less boards (linux, zephyr,
            # freertos-posix) state nothing, which is the right answer.
            "board_crate": None,
        },
    },
}

# Fields whose derivation needs the PACKAGE rather than the canonical name.
# `None` in a family's field map means "look here"; the convention functions
# above stay pure functions of the name.
PACKAGE_DERIVED = {"crate": own_crate_name, "board_crate": own_crate_name}
assert all(
    (conv is None) == (field in PACKAGE_DERIVED or field in ENTRY_DERIVED)
    for fam in FAMILIES.values()
    for field, conv in fam["fields"].items()
), "a field is derived from the name OR from the package, never neither/both"


def rel(p):
    return str(Path(p).relative_to(ROOT)) if Path(p).is_absolute() else str(p)


# ---------------------------------------------------------------------------
# The scan
# ---------------------------------------------------------------------------
def scan(root):
    """-> (rows, checked_fields, notes).

    A row is `(key, stated, derived_or_UNAVAILABLE, why)`: a divergence, or a
    stated field whose derivation is unavailable (D2).
    """
    rows, checked, notes = [], 0, []
    for kind, fam in sorted(FAMILIES.items()):
        globs = fam["glob"]
        globs = [globs] if isinstance(globs, str) else list(globs)
        paths = sorted({p for g in globs for p in Path(root).glob(g)})
        if not paths:
            notes.append(
                f"family {kind!r}: no descriptor matched {globs!r} — "
                f"refusing to pass on an empty set"
            )
            continue
        for desc in paths:
            key_path = str(desc.relative_to(root))
            try:
                data = tomllib.loads(desc.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                notes.append(f"{key_path}: not valid TOML: {exc}")
                continue

            pkg_dir = desc.parent
            announced = announced_names(pkg_dir, kind)
            entries = fam["entries"](data)
            # The canonical name is the FIRST announced one. With several
            # entries in one package the flat announcement cannot say which
            # names belong to which entry (nros-board-nuttx-qemu: 2 entries, 7
            # names), so a per-entry canonical name is unavailable — and every
            # field derived from it with it.
            attributable = announced is not None and len(entries) == 1

            for label, entry in entries:
                for field, conv in fam["fields"].items():
                    if field not in entry:
                        continue  # not stated: the whole point
                    stated = entry[field]
                    checked += 1
                    key = f"{key_path}@{label}.{field}"
                    if field in ENTRY_DERIVED:
                        derived = ENTRY_DERIVED[field](entry)
                        derived = UNAVAILABLE if derived is None else derived
                    elif field in PACKAGE_DERIVED:
                        derived = PACKAGE_DERIVED[field](pkg_dir)
                        derived = UNAVAILABLE if derived is None else derived
                    elif attributable and announced:
                        derived = conv(kind, announced[0])
                    else:
                        derived = UNAVAILABLE
                    if derived is UNAVAILABLE:
                        why = (
                            "states a derivable field whose derivation is "
                            "unavailable here — nothing can check it"
                        )
                        rows.append((key, stated, UNAVAILABLE, why))
                    elif stated != derived:
                        why = (
                            f"states {stated!r} but convention derives "
                            f"{derived!r}"
                        )
                        rows.append((key, stated, derived, why))
    return rows, checked, notes


# ---------------------------------------------------------------------------
# Baseline — the ratchet
# ---------------------------------------------------------------------------
def load_baseline(errors):
    if not BASELINE.exists():
        errors.append(
            f"missing baseline {rel(BASELINE)} — regenerate with "
            "--write-baseline. A missing baseline is a FAILURE, not an empty "
            "exemption set: read as empty it would fail every grandfathered "
            "row at once, with a message about derivation rather than about "
            "the missing file, and invite the bypass the ratchet exists to "
            "avoid"
        )
        return {}
    try:
        return json.loads(BASELINE.read_text())
    except (OSError, ValueError) as exc:
        errors.append(f"cannot read baseline {rel(BASELINE)}: {exc}")
        return {}


def write_baseline(rows):
    out = {key: stated for key, stated, _d, _w in rows}
    BASELINE.write_text(json.dumps(dict(sorted(out.items())), indent=2) + "\n")
    print(f"wrote {rel(BASELINE)}: {len(out)} divergence(s) grandfathered")
    return 0


def apply_baseline(rows, baseline, root):
    """-> (errors, exempt, improved)."""
    errors, exempt, seen = [], [], set()
    for key, stated, _derived, why in rows:
        seen.add(key)
        if key not in baseline:
            errors.append(f"{key}: {why} (RFC-0087 D4)")
        elif baseline[key] != stated:
            errors.append(
                f"{key}: grandfathered as {baseline[key]!r} but now states "
                f"{stated!r} — a different divergence is not the one that was "
                f"granted; fix it, do not re-baseline it"
            )
        else:
            exempt.append(key)

    improved = []
    for key in sorted(set(baseline) - seen):
        desc = key.split("@", 1)[0]
        if not (Path(root) / desc).is_file():
            errors.append(
                f"baseline names {key}, whose descriptor does not exist — a "
                f"stale exemption is how a rule gets reclaimed by accident; "
                f"drop it (--write-baseline)"
            )
        else:
            improved.append(key)
    return errors, exempt, improved


# ---------------------------------------------------------------------------
# Negative controls — every rule is fired, then silenced by its escape.
# ---------------------------------------------------------------------------
def _rmw_pkg(root, name, descriptor, announce=("rmw",), cargo_name=None):
    d = Path(root) / "packages/rmw" / name / f"nros-rmw-{name}"
    d.mkdir(parents=True, exist_ok=True)
    (d / "nros-rmw.toml").write_text(descriptor)
    if announce is not None:
        tags = "\n".join(
            f'    <nano_ros_provides kind="rmw" name="{n}"/>' for n in announce
        )
        (d / "package.xml").write_text(
            f'<package format="3"><name>p</name>\n  <export>\n{tags}\n'
            f"  </export>\n</package>\n"
        )
    if cargo_name:
        (d / "Cargo.toml").write_text(f'[package]\nname = "{cargo_name}"\n')
    return str((d / "nros-rmw.toml").relative_to(root))


def _board_pkg(root, name, descriptor, announce=(), cargo_name=None):
    d = Path(root) / "packages/boards" / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "nros-board.toml").write_text(descriptor)
    tags = "\n".join(
        f'    <nano_ros_provides kind="board" name="{n}"/>' for n in announce
    )
    (d / "package.xml").write_text(
        f'<package format="3"><name>p</name>\n  <export>\n{tags}\n'
        f"  </export>\n</package>\n"
    )
    if cargo_name:
        (d / "Cargo.toml").write_text(f'[package]\nname = "{cargo_name}"\n')
    return str((d / "nros-board.toml").relative_to(root))


def self_test(quiet=False):
    """A gate nobody has watched fail is a gate that has not been shown to
    have a failing mode. Each rule fires here on a constructed input."""
    with tempfile.TemporaryDirectory() as td:
        # A clean tree: one nameless rmw descriptor, one board.
        good_rmw = _rmw_pkg(td, "good", '[rmw]\ncpp_define = "X"\n')
        good_board = _board_pkg(
            td, "nros-board-good", '[[board]]\nboard_crate = "nros-board-good"\n',
            announce=("good",), cargo_name="nros-board-good",
        )
        rows, checked, notes = scan(td)
        assert not notes, notes
        assert not rows, rows
        assert checked == 1, checked  # only board_crate is stated

        # D1 — a restated derivable field that disagrees. Two of them, so the
        # rule is seen firing per FIELD and not merely per descriptor.
        bad = _rmw_pkg(
            td, "bad",
            '[rmw]\ncpp_define = "X"\ncargo_feature = "rmw-typo"\n'
            'c_define_token = "WRONG"\n',
            announce=("bad",),
        )
        rows, _c, _n = scan(td)
        keys = {k for k, *_ in rows}
        assert f"{bad}@rmw.cargo_feature" in keys, keys
        assert f"{bad}@rmw.c_define_token" in keys, keys
        errs, _e, _i = apply_baseline(rows, {}, td)
        assert len(errs) == 2, errs

        # …silenced by stating the derived value instead. Note `BAD` would be
        # accepted, because it IS `UPPER("bad")`: the rule is equality with the
        # convention, not "must be absent".
        (Path(td) / bad).write_text(
            '[rmw]\ncpp_define = "X"\ncargo_feature = "rmw-bad"\n'
            'c_define_token = "BAD"\n'
        )
        rows, _c, _n = scan(td)
        assert not [r for r in rows if r[0].startswith(f"{bad}@")], rows
        # Restore one divergence for the baseline rules below.
        (Path(td) / bad).write_text(
            '[rmw]\ncpp_define = "X"\ncargo_feature = "rmw-typo"\n'
        )

        # D2 — a stated field whose derivation is unavailable. `crate` on a
        # package with no Cargo.toml is exactly `nros-rmw-xrce`'s shape.
        nocargo = _rmw_pkg(
            td, "nocargo",
            '[rmw]\ncpp_define = "X"\n\n[rmw.provides.cargo]\n'
            'crate = "some-sibling-cffi"\n',
            announce=("nocargo",),
        )
        rows, _c, _n = scan(td)
        row = next(r for r in rows if r[0] == f"{nocargo}@rmw.crate")
        assert row[2] is UNAVAILABLE, row
        # …silenced by the package actually owning that crate.
        (Path(td) / nocargo).parent.joinpath("Cargo.toml").write_text(
            '[package]\nname = "some-sibling-cffi"\n'
        )
        rows, _c, _n = scan(td)
        assert f"{nocargo}@rmw.crate" not in {k for k, *_ in rows}

        # A multi-entry board: the flat announcement cannot be attributed, so
        # a name-derived field is UNAVAILABLE rather than silently wrong.
        multi = _board_pkg(
            td, "nros-board-multi",
            '[[board]]\nboard_crate = "nros-board-multi"\n\n'
            '[[board]]\nboard_crate = "wrong-crate"\n',
            announce=("a", "b"), cargo_name="nros-board-multi",
        )
        rows, _c, _n = scan(td)
        keys = {k for k, *_ in rows}
        assert f"{multi}@board[1].board_crate" in keys, keys
        assert f"{multi}@board[0].board_crate" not in keys, keys

        # D3 — the ratchet, in both directions.
        rows, _c, _n = scan(td)
        base = {k: s for k, s, _d, _w in rows}
        errs, exempt, improved = apply_baseline(rows, base, td)
        assert not errs and len(exempt) == len(rows) and not improved

        # a DIFFERENT divergence at a grandfathered key is a new violation
        swapped = dict(base)
        swapped[f"{bad}@rmw.cargo_feature"] = "rmw-some-other-typo"
        errs, _e, _i = apply_baseline(rows, swapped, td)
        assert len(errs) == 1 and "do not re-baseline" in errs[0], errs

        # fixing a row is REPORTED, never failed
        errs, _e, improved = apply_baseline([], base, td)
        assert not errs and improved == sorted(base), (errs, improved)

        # a baseline row whose descriptor is gone is an ERROR
        errs, _e, _i = apply_baseline([], {"packages/rmw/x/y/nros-rmw.toml@rmw.crate": "z"}, td)
        assert len(errs) == 1 and "does not exist" in errs[0], errs

        # D4 — vacuity. An empty tree must fail, not print OK.
        with tempfile.TemporaryDirectory() as empty:
            _rows, _c, notes = scan(empty)
            assert len(notes) == len(FAMILIES), notes

        assert good_rmw and good_board  # constructed, and clean

    # A missing baseline must FAIL rather than read as an empty exemption set.
    global BASELINE
    real, BASELINE = BASELINE, ROOT / "scripts/does-not-exist.json"
    try:
        errs = []
        assert load_baseline(errs) == {} and errs, (
            "a missing baseline must be an error, not a silent empty set")
    finally:
        BASELINE = real

    if not quiet:
        print("check-derived-descriptor-fields self-test: OK")
    return 0


# ---------------------------------------------------------------------------
def main():
    argv = sys.argv[1:]
    if "--self-test" in argv:
        return self_test()
    # Always, not only behind the flag: a negative control nobody runs decays
    # into a comment, and this rule's whole job is to fire.
    self_test(quiet=True)

    scan_root = None
    if "--scan-root" in argv:
        scan_root = argv[argv.index("--scan-root") + 1]
    root = scan_root or ROOT

    rows, checked, notes = scan(root)
    errors = list(notes)

    if "--list" in argv:
        for key, stated, derived, _why in rows:
            shown = "(underivable)" if derived is UNAVAILABLE else repr(derived)
            print(f"{key}\tstated={stated!r}\tderived={shown}")
        print(f"{checked} stated derivable field(s) examined")
        return 0
    if "--write-baseline" in argv:
        if scan_root:
            print("--write-baseline works on the repo, not --scan-root",
                  file=sys.stderr)
            return 2
        return write_baseline(rows)

    baseline = {} if scan_root else load_baseline(errors)
    errs, exempt, improved = apply_baseline(rows, baseline, root)
    errors += errs

    # D4 — a scan that examined nothing is not a pass.
    if checked == 0 and not errors:
        errors.append(
            "no descriptor states a single derivable field — this gate would "
            "be vacuous. That is a legitimate END STATE, but it must be made "
            "deliberately: delete this gate, or say here why zero is right"
        )

    print(
        f"{checked} stated derivable field(s) across "
        f"{len(FAMILIES)} provider famil(ies)"
    )
    if exempt:
        print(
            f"  {len(exempt)} grandfathered by {rel(BASELINE)} — the list only "
            "shrinks; a NEW descriptor binds immediately"
        )
    for key in improved:
        print(f"  {key} no longer diverges — drop it from the baseline "
              "(--write-baseline)")

    if errors:
        print("\n[FAIL] derived descriptor fields (RFC-0087 D4):",
              file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 1
    print("Every stated derivable field equals the value convention derives.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
