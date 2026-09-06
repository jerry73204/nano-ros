#!/usr/bin/env python3
"""Static-memory report for a built nano-ros image — phase 392 W1, issue 0815.

Phase 392 opened with a table of `nm` output pasted into a markdown file: 27% of
a safety-island image was message buffers, and the largest consumers were pools
the inventory could not price. Every later wave of that campaign is defined as a
saving against those numbers ("W3 requires W1 so the saving is measured rather
than asserted"), so the numbers have to come from a tool that anyone can re-run
against any image, not from a paste that ages.

This is that tool. It reads an ELF's symbol table, attributes every byte of RAM
to a crate and — where the pool declares its arithmetic — to a named pool, and
reports the total against the section sizes so nothing hides in the gap.

Why measured rather than declared
---------------------------------
`scripts/gen-pool-inventory.py` prices a pool from a `// nros-pool:` comment
evaluated at the knobs' defaults. That works for a pool of BYTES and stops at a
pool of STRUCTS, for three independent reasons, all of them the same reason:

  * `SERVICE_BUFFERS` is `ZPICO_MAX_SESSIONS * ZPICO_MAX_QUERYABLES *
    sizeof(ServiceBuffer)`, and `ZPICO_MAX_QUERYABLES` has a COMPUTED default —
    there is no integer to put in the comment.
  * `MESSAGE_INFO_TABLE`'s element gains three fields under `alloc` +
    `safety-e2e`, so a constant would be right for one build and wrong for the
    rest. Issue 0739 declined to annotate it for exactly that reason, and was
    right to.
  * `__nros_comp_buf_N` is emitted by codegen as `sizeof(component class)`.

The size is known to the COMPILER, not to a comment. So read it from the
compiler's output. A hand-written figure in a comment is also the drift class
this tree already has gates against (`check-ffi-struct-mirrors`): it is correct
until someone appends a field.

The two instruments compose. `--check` joins the declared arithmetic to the
measured symbol and asserts they agree on an image built at knob defaults, which
turns the inventory's numbers from a claim into a checked fact.

What this tool cannot see
-------------------------
It reports STORAGE, never REFERENCES. `--size-sort` drops sizeless symbols, and
an undefined reference into libc — the `U malloc` that proves an image reaches
the heap — is exactly a sizeless symbol. So a green report here says nothing
about whether the image allocates. That question has its own tool:
`scripts/check-no-alloc-image.py` (issue 0816), which reads the symbol table
without `--size-sort` for that reason.

The unattributed gap in the report is the other half of the same honesty: symbols
never sum to the section size, so the difference is printed rather than left for
someone to discover when their budget comes up short.

Usage
-----
    scripts/nros-mem-report.py <elf> [<elf>...]      # human report
    scripts/nros-mem-report.py <elf> --json          # machine readable
    scripts/nros-mem-report.py <elf> --check         # declared == measured
    scripts/nros-mem-report.py <elf> --baseline b.json   # deltas vs a baseline

`--check` assumes the image was built at knob DEFAULTS; point it at a fixture,
not at a board build that tunes them.
"""

import argparse
import contextlib
import importlib.util
import io
import json
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# nm type letters. Lowercase is local, uppercase global; the CLASS is the same.
RAM_TYPES = set("bBdDgGsS")  # .bss + .data + small-data variants
ROM_TYPES = set("rR")  # .rodata
TEXT_TYPES = set("tTwWiI")  # .text, weak, indirect

NM_LINE = re.compile(r"^([0-9]+)\s+([0-9]+)\s+(\S)\s+(.*)$")

# The leading `crate::` of a demangled Rust symbol. Handles the plain form
# (`nros_rmw_zenoh::shim::...`) and the qualified one (`<T as Trait>::m`), where
# the first identifier inside the brackets is the one that owns the bytes.
CRATE = re.compile(r"\b([a-z][a-z0-9_]*)::")


def load_inventory():
    """Import gen-pool-inventory for its knob/pool scanner (hyphenated name)."""
    path = os.path.join(ROOT, "scripts", "gen-pool-inventory.py")
    spec = importlib.util.spec_from_file_location("gen_pool_inventory", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def rustlib_bin(name):
    """`name` inside the active toolchain's llvm-tools, or None.

    rustup ships llvm-nm/llvm-size under
    `<sysroot>/lib/rustlib/<host>/bin/` and does NOT put them on PATH, so a
    contributor who can cross-build for Cortex-M already has the one tool that
    can read the result and does not know it. Look there before telling anyone
    to install a system LLVM.
    """
    try:
        sysroot = subprocess.run(
            ["rustc", "--print", "sysroot"], capture_output=True, text=True, check=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    rustlib = os.path.join(sysroot, "lib", "rustlib")
    if not os.path.isdir(rustlib):
        return None
    for host in sorted(os.listdir(rustlib)):
        cand = os.path.join(rustlib, host, "bin", name)
        if os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    return None


def find_tool(name):
    return shutil.which(name) or rustlib_bin(name)


def pick_nm(elf):
    """An `nm` that can read THIS file.

    GNU nm is built for one target family and refuses a foreign ELF with "File
    format not recognized" — which is most of what we want to measure, since the
    images that care about RAM are the cross-built ones. llvm-nm reads them all,
    so prefer it and fall back to the toolchain-prefixed and plain names.
    """
    candidates = ["llvm-nm", "arm-none-eabi-nm", "nm"]
    for name in candidates:
        tool = find_tool(name)
        if not tool:
            continue
        probe = subprocess.run(
            [tool, "--print-size", elf], capture_output=True, text=True
        )
        if probe.returncode == 0:
            return tool
    raise SystemExit(
        f"no usable nm for {elf} — tried {', '.join(candidates)}. "
        "For a cross-built image, add rustup's llvm-tools "
        "(`rustup component add llvm-tools`) — it ships an llvm-nm this script "
        "finds without needing it on PATH."
    )


def read_symbols(elf):
    """[(size, type, demangled_name)] for every sized symbol in the image."""
    tool = pick_nm(elf)
    out = subprocess.run(
        [tool, "-C", "--print-size", "--size-sort", "--radix=d", elf],
        capture_output=True,
        text=True,
    )
    if out.returncode != 0:
        raise SystemExit(f"{tool} failed on {elf}: {out.stderr.strip()}")
    syms = []
    for line in out.stdout.splitlines():
        m = NM_LINE.match(line)
        if not m:
            continue
        _addr, size, typ, name = m.groups()
        syms.append((int(size), typ, name.strip()))
    return syms


def read_sections(elf):
    """{section: bytes} from `size -A`, the authoritative totals.

    Symbols never sum to the section size — alignment padding, linker-script
    reservations and symbol-less data all live in the gap. Reporting the gap is
    the point: a campaign that only counts what it can name will keep finding
    the image bigger than its own table.
    """
    tool = find_tool("llvm-size") or find_tool("size")
    if not tool:
        return {}
    out = subprocess.run([tool, "-A", elf], capture_output=True, text=True)
    if out.returncode != 0:
        return {}
    sections = {}
    for line in out.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith(".") and parts[1].isdigit():
            sections[parts[0]] = int(parts[1])
    return sections


def crate_ident_for(rel, inv):
    """The Rust crate identifier that owns a source file, or None.

    A pool is joined to its symbol by crate AND leaf name, never by leaf alone.
    `SLOTS` is declared in `nros-rmw-cffi`, and `nros_log::early::SLOTS` is a
    different 1,440-byte pool that merely shares the last path segment — joining
    on the leaf reported a 6,752-byte "drift" in an image that contains neither
    the crate nor the pool.
    """
    crate_dir = inv.crate_of(rel)
    manifest = os.path.join(ROOT, crate_dir, "Cargo.toml")
    try:
        with open(manifest, encoding="utf8") as fh:
            for line in fh:
                m = re.match(r'\s*name\s*=\s*"([^"]+)"', line)
                if m:
                    return m.group(1).replace("-", "_")
    except OSError:
        return None
    return None


def newest_source_mtime(crate_dirs):
    """mtime of the newest tracked source under the given crate dirs, or None.

    Scoped to the crates that DECLARE the pools being checked, not to all of
    packages/. A number is only as good as the artifact it came from, and this
    tool is pointed at a path typed by hand -- which the harness's staleness
    probe does not guard, because that probe covers fixtures the harness
    RESOLVES. Issue 0827's first draft was measured on a three-week-old binary
    in `examples/**/target-*/`, the pre-phase-340 layout that
    `build-test-fixtures` no longer writes.

    The scope matters as much as the check. A first version compared against
    every tracked source under packages/, which made an unrelated edit -- two
    C test TUs in `nros-c` -- report a perfectly fresh zenoh fixture as stale.
    A staleness rule that fires on files the artifact does not depend on gets
    switched off, and then it guards nothing.
    """
    if not crate_dirs:
        return None
    patterns = []
    for d in sorted(crate_dirs):
        patterns += [f"{d}/*.rs", f"{d}/*.c", f"{d}/*.cpp", f"{d}/*.h", f"{d}/*.hpp"]
    try:
        files = subprocess.run(
            ["git", "ls-files"] + patterns,
            cwd=ROOT, capture_output=True, text=True, check=True,
        ).stdout.split()
    except (OSError, subprocess.CalledProcessError):
        return None
    newest = None
    for rel in files:
        try:
            m = os.path.getmtime(os.path.join(ROOT, rel))
        except OSError:
            continue
        if newest is None or m > newest:
            newest = m
    return newest


def crate_of(name):
    """The crate that owns a symbol, or a bucket for the ones with no path."""
    m = CRATE.search(name)
    if m:
        return m.group(1)
    return "(C / asm / no path)"


def analyse(elf, pools_by_name):
    syms = read_symbols(elf)
    sections = read_sections(elf)

    ram = [(s, t, n) for s, t, n in syms if t in RAM_TYPES]
    rom = [(s, t, n) for s, t, n in syms if t in ROM_TYPES]
    text = [(s, t, n) for s, t, n in syms if t in TEXT_TYPES]

    by_crate = {}
    for size, _t, name in ram:
        by_crate[crate_of(name)] = by_crate.get(crate_of(name), 0) + size

    # Join measured symbols to declared pools by LAST path segment: a pool is
    # `nros_rmw_zenoh::shim::service::SERVICE_BUFFERS` in the image and
    # `SERVICE_BUFFERS` in the annotation.
    matched = []
    matched_crate_dirs = set()
    for size, _t, name in ram:
        leaf = name.rsplit("::", 1)[-1]
        if leaf in pools_by_name:
            expr, declared, err, rel, line, crate, crate_dir = pools_by_name[leaf]
            # Same leaf in a different crate is a different pool.
            if crate is not None and not name.startswith(crate + "::"):
                continue
            if crate_dir:
                matched_crate_dirs.add(crate_dir)
            matched.append(
                {
                    "pool": leaf,
                    "symbol": name,
                    "measured": size,
                    "declared": declared,
                    "formula": expr,
                    "unpriced_because": err,
                    "declared_at": f"{rel}:{line}",
                }
            )

    section_ram = sum(
        v for k, v in sections.items() if k.startswith((".bss", ".data", ".sbss", ".sdata"))
    )
    try:
        elf_mtime = os.path.getmtime(elf)
    except OSError:
        elf_mtime = None
    newest_src = newest_source_mtime(matched_crate_dirs)
    stale = (
        elf_mtime is not None and newest_src is not None and newest_src > elf_mtime
    )
    return {
        "elf": os.path.relpath(elf, ROOT) if elf.startswith(ROOT) else elf,
        "elf_mtime": elf_mtime,
        "newest_source_mtime": newest_src,
        "stale": stale,
        "ram_symbol_total": sum(s for s, _, _ in ram),
        "rodata_symbol_total": sum(s for s, _, _ in rom),
        "text_symbol_total": sum(s for s, _, _ in text),
        "section_ram_total": section_ram,
        "sections": sections,
        "by_crate": by_crate,
        "top_ram": [
            {"bytes": s, "symbol": n} for s, _t, n in sorted(ram, reverse=True)[:40]
        ],
        "pools": matched,
    }


def fmt(n):
    return f"{n:,}"


def report(res, top, baseline=None):
    lines = []
    add = lines.append
    add(f"# static memory — {res['elf']}")
    add("")
    if res.get("stale"):
        add(
            "!! STALE IMAGE — a tracked source under packages/ is NEWER than this"
        )
        add(
            "   artifact, so every number below describes code that is no longer"
        )
        add(
            "   in the tree. Rebuild before quoting any of it — with the recipe"
        )
        add(
            "   that PRODUCES this artifact, which differs by lane:"
        )
        add("     native fixtures      just build-test-fixtures lane=native")
        add("                          (measure under build/cargo-fixtures/, NOT")
        add("                           examples/**/target-*/ — the pre-phase-340")
        add("                           layout, which nothing rewrites)")
        add("     cross link-check     just rust-rtos-link-check")
        add("")
    ram_sym, ram_sec = res["ram_symbol_total"], res["section_ram_total"]
    add(f"RAM (.bss + .data), by section:  {fmt(ram_sec)} bytes")
    add(f"RAM attributed to symbols:       {fmt(ram_sym)} bytes")
    if ram_sec:
        gap = ram_sec - ram_sym
        add(
            f"unattributed (padding, linker reservations, symbol-less data): "
            f"{fmt(gap)} bytes ({100.0 * gap / ram_sec:.1f}%)"
        )
    add(f"rodata in symbols:               {fmt(res['rodata_symbol_total'])} bytes")
    add(f"text in symbols:                 {fmt(res['text_symbol_total'])} bytes")
    add("")

    add(f"## top {top} RAM symbols")
    add("")
    base_syms = {}
    if baseline:
        base_syms = {t["symbol"]: t["bytes"] for t in baseline.get("top_ram", [])}
    for row in res["top_ram"][:top]:
        share = 100.0 * row["bytes"] / ram_sec if ram_sec else 0.0
        delta = ""
        if row["symbol"] in base_syms:
            d = row["bytes"] - base_syms[row["symbol"]]
            if d:
                delta = f"  ({d:+,})"
        add(f"  {fmt(row['bytes']):>12}  {share:5.1f}%  {row['symbol']}{delta}")
    add("")

    add("## RAM by crate")
    add("")
    for crate, size in sorted(res["by_crate"].items(), key=lambda kv: -kv[1])[:20]:
        share = 100.0 * size / ram_sec if ram_sec else 0.0
        add(f"  {fmt(size):>12}  {share:5.1f}%  {crate}")
    add("")

    add("## declared pools, measured")
    add("")
    if not res["pools"]:
        add("  (no annotated pool is linked into this image)")
    for p in res["pools"]:
        if p["declared"] is None:
            add(
                f"  {fmt(p['measured']):>12}  {p['pool']}  "
                f"— measured only; not priceable statically ({p['unpriced_because']})"
            )
        elif p["declared"] == p["measured"]:
            add(f"  {fmt(p['measured']):>12}  {p['pool']}  — agrees with `{p['formula']}`")
        else:
            add(
                f"  {fmt(p['measured']):>12}  {p['pool']}  — DECLARED "
                f"{fmt(p['declared'])} at defaults, formula `{p['formula']}`"
            )
    return "\n".join(lines)


def check(res):
    """Declared arithmetic must equal the measured symbol on a default build."""
    if res.get("stale"):
        print(
            f"check-mem-report: {res['elf']} is STALE — a tracked source is newer\n"
            "than the artifact, so comparing declared arithmetic against it proves\n"
            "nothing about the current tree. Rebuild and re-run."
        )
        return 1
    priced = [p for p in res["pools"] if p["declared"] is not None]
    if not res["pools"]:
        # A check with nothing to check reads as coverage and is not. Every
        # image this is pointed at links a backend, and every backend has at
        # least one annotated pool — so zero MATCHES means the join broke (a
        # renamed symbol, a stripped binary, a demangler that did not run),
        # not that the image is lean.
        print(
            f"check-mem-report: NO annotated pool is linked into {res['elf']} — "
            "the check would be vacuous.\n"
            "Either the image links no RMW backend, or the symbol-to-annotation\n"
            "join broke (stripped binary, renamed pool, demangling off)."
        )
        return 1
    if not priced:
        # MATCHED but not PRICEABLE, which is a different state and not an
        # error. The join worked -- these pools are in the image and were
        # identified -- but their formulas multiply a knob whose default is
        # computed rather than literal, so there is no static number to
        # compare against.
        #
        # Conflating the two cost `queue.yml` every run it ever made. The
        # nuttx talker links exactly `SMALL_PAYLOADS` and `LARGE_PAYLOADS`,
        # both priced through `ZPICO_SUBSCRIBER_RING_DEPTH`, whose default is
        # computed (issue 0829's SYSTEM_DEFAULT sentinel) -- so `priced` was
        # empty and the image was reported as linking no backend, which it
        # plainly does. The freertos talker passes only because it also links
        # `nros_rmw_cffi`'s `SLOTS`, whose formula is literal.
        #
        # Reported, not silent: an unpriceable pool is a gap in what this tool
        # can assert, and a lane that prints nothing about it invites the same
        # misreading from the other direction.
        print(
            f"check-mem-report: {len(res['pools'])} annotated pool(s) linked into "
            f"{res['elf']}, none PRICEABLE.\n"
            "The join worked -- these are the pools, measured -- but every formula\n"
            "multiplies a knob with a computed default, so there is no static\n"
            "number to check the arithmetic against:\n"
        )
        for p in res["pools"]:
            print(f"  {fmt(p['measured']):>12}  {p['pool']}  ({p['declared_at']})")
            print(f"                formula   {p['formula']}")
            print(f"                unpriced  {p['unpriced_because']}")
        print(
            "\nNot a failure: the check asserts arithmetic it cannot compute here.\n"
            "Give one of these knobs a literal default to make the pool checkable."
        )
        return 0
    bad = [p for p in priced if p["declared"] != p["measured"]]
    if not bad:
        print(
            f"check-mem-report: {len(priced)} declared pool(s) agree with "
            f"{res['elf']}"
        )
        return 0
    print(f"check-mem-report: {len(bad)} declared pool(s) disagree with the image\n")
    for p in bad:
        print(f"  {p['pool']}  ({p['declared_at']})")
        print(f"    formula   {p['formula']}")
        print(f"    declared  {fmt(p['declared'])} bytes at knob defaults")
        print(f"    measured  {fmt(p['measured'])} bytes in {res['elf']}")
        print("")
    print(
        "Either the formula drifted from the type it prices (a field was\n"
        "appended, or an element size changed), or this image was NOT built at\n"
        "knob defaults. `--check` assumes defaults; point it at a fixture."
    )
    return 1


def selftest():
    """The gate has to be able to FAIL, or a green means nothing.

    Same reasoning as the other generated-page checks in this tree: an
    always-passing check is worse than no check, because it reads as coverage.
    """
    agreeing = {
        "elf": "synthetic",
        "pools": [
            {
                "pool": "P",
                "declared": 1024,
                "measured": 1024,
                "formula": "K * 8",
                "declared_at": "x.rs:1",
            }
        ],
    }
    drifted = {
        "elf": "synthetic",
        "pools": [
            {
                "pool": "P",
                "declared": 1024,
                "measured": 2048,
                "formula": "K * 8",
                "declared_at": "x.rs:1",
            }
        ],
    }
    unpriceable = {
        "elf": "synthetic",
        "pools": [
            {
                "pool": "P",
                "declared": None,
                "measured": 2048,
                "formula": "K * SZ",
                "declared_at": "x.rs:1",
                "unpriced_because": "knob `K` has a computed default",
            }
        ],
    }
    empty = {"elf": "synthetic", "pools": []}
    stale = {
        "elf": "synthetic",
        "stale": True,
        "pools": [
            {
                "pool": "P",
                "declared": 1024,
                "measured": 1024,
                "formula": "K * 8",
                "declared_at": "x.rs:1",
            }
        ],
    }

    def quiet(case):
        # The synthetic failures print their full operator report. Swallow it:
        # a gate whose green output contains four fake failure reports is a
        # gate whose red nobody will spot.
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            return check(case)

    # An unpriceable pool BESIDE a drifted one must still fail on the drift:
    # the tolerance is for "nothing to compare", never for "did not compare".
    mixed = {
        "elf": "synthetic",
        "pools": [unpriceable["pools"][0], drifted["pools"][0]],
    }

    assert quiet(agreeing) == 0, "agreeing pool must pass"
    assert quiet(drifted) == 1, "drifted pool must FAIL"
    assert quiet(unpriceable) == 0, (
        "a pool that MATCHED but cannot be priced is not a failure — the join "
        "worked; there is simply no static number to compare. Conflating this "
        "with the empty case is what kept queue.yml red on every run"
    )
    assert quiet(mixed) == 1, "an unpriceable pool must not mask a drifted one"
    assert quiet(empty) == 1, "a vacuous check must FAIL, not read as coverage"
    assert quiet(stale) == 1, "an agreeing pool on a STALE image must still FAIL"
    print("selftest: ok — the check passes on agreement and fails on drift")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("elf", nargs="*", help="built image(s) to measure")
    ap.add_argument("--top", type=int, default=25, help="how many RAM symbols to list")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    ap.add_argument(
        "--check",
        action="store_true",
        help="assert declared pool arithmetic equals the measured symbol",
    )
    ap.add_argument("--baseline", help="a --json file to show deltas against")
    ap.add_argument(
        "--selftest", action="store_true", help="prove the check can fail"
    )
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.elf:
        ap.error("give at least one ELF, or --selftest")

    inv = load_inventory()
    knobs, pools = inv.scan()
    pools_by_name = {}
    for name, expr, rel, line in pools:
        b, err = inv.pool_bytes(expr, knobs)
        pools_by_name[name] = (
            expr, b, err, rel, line, crate_ident_for(rel, inv), inv.crate_of(rel),
        )

    baseline = None
    if args.baseline:
        with open(args.baseline, encoding="utf8") as fh:
            baseline = json.load(fh)
        if isinstance(baseline, list):
            baseline = baseline[0]

    results = [analyse(os.path.abspath(e), pools_by_name) for e in args.elf]

    if args.json:
        print(json.dumps(results if len(results) > 1 else results[0], indent=2))
        return 0

    rc = 0
    for res in results:
        if args.check:
            rc |= check(res)
        else:
            print(report(res, args.top, baseline))
            print("")
    return rc


if __name__ == "__main__":
    sys.exit(main())
