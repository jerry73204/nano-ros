#!/usr/bin/env python3
"""A board's `cargo_config` blob is real TOML, of a shape cargo understands.

WHY THIS EXISTS. `cargo_config` in `nros-board.toml` is a TOML document inside a
TOML document: the descriptor holds it as a triple-quoted string, and the CLI
substitutes `${workspace}` and writes it into a leaf's `.cargo/config.toml`.
Nothing checked the inner document. A typo — `runnner`, `[targets.x]`,
`rustflag` — travels all the way into a generated file where cargo silently
ignores it, and the leaf then builds for the wrong target or links without the
flags it needed.

phase-375 W8 asked for the blob to be DECOMPOSED into `target` / `runner` /
`rustflags` / `linker` fields instead. Measured across the seven blobs in the
tree, that is the wrong shape: they also carry `[unstable]` (`build-std`),
`[env]` (`CC_armv7a_nuttx_eabihf`, `THREADX_PORT`, `NETX_CONFIG_DIR`, `ESP_LOG`)
and `[patch.crates-io]`. Four fields would cover part of every blob and none of
them completely, so each board would end up with BOTH fields and a blob — two
places to look instead of one, which is worse than the problem.

So the blob stays, and gets what it was actually missing: a schema check.

WHAT IT CHECKS, per blob:

  * it parses as TOML at all;
  * every table is one cargo reads from a config file (`[build]`, `[target.*]`,
    `[env]`, `[unstable]`, `[patch.*]`, `[alias]`, `[net]`, `[term]`,
    `[profile.*]`, `[registries.*]`, `[source.*]`);
  * inside `[target.*]`, every key is one cargo defines there — this is where
    `runner` / `rustflags` / `linker` live and where a typo is invisible;
  * `[build] target`, when present, names a target the blob also configures, or
    is a bare triple. A `[build] target` pointing at a `[target.X]` that does
    not exist is the specific shape that builds for the wrong triple.

Exit 0 when every blob is well-shaped, 1 otherwise.
"""

import glob
import os
import re
import sys
import tempfile

try:
    import tomllib
except ModuleNotFoundError:  # 3.10 backport, same spelling as the sibling gates
    import tomli as tomllib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

BOARD_GLOBS = [
    "packages/boards/*/nros-board.toml",
    "packages/boards/*/boards/*/nros-board.toml",
]

# Tables cargo reads from `.cargo/config.toml`. Prefix entries match a dotted
# family (`target.thumbv7m-none-eabi`, `patch.crates-io`).
# tomllib NESTS, so `[target.thumbv7m-none-eabi]` arrives as
# `data["target"]["thumbv7m-none-eabi"]` and the top-level key is always a bare
# name. An earlier draft matched prefixes and let `[targets.x]` through, because
# `"targets".startswith("target")` — the typo this gate exists to catch, passing
# the check meant to catch it.
CARGO_TABLES = {
    "build", "env", "unstable", "alias", "net", "term", "doc",
    "future-incompat-report", "target", "patch", "profile", "registries",
    "source", "http", "credential-alias", "install", "cargo-new", "resolver",
}

# Keys cargo defines inside `[target.<triple>]`.
TARGET_KEYS = {"linker", "runner", "rustflags", "rustdocflags", "ar"}

CARGO_CONFIG_RE = re.compile(r"cargo_config\s*=\s*'''(.*?)'''", re.S)


def blob_of(text):
    m = CARGO_CONFIG_RE.search(text)
    return m.group(1) if m else None


def check_blob(rel, blob):
    problems = []
    # `${workspace}` is the CLI's placeholder and is not valid TOML in a bare
    # position; it only ever appears inside strings here, so parsing is safe.
    try:
        data = tomllib.loads(blob)
    except Exception as e:  # noqa: BLE001 — report, do not raise
        return [f"{rel}: `cargo_config` is not valid TOML: {e}"]

    for table, body in data.items():
        if table not in CARGO_TABLES:
            problems.append(
                f"{rel}: `cargo_config` has a `[{table}]` table, which cargo does "
                f"not read from a config file. It will be silently ignored."
            )
            continue
        if table == "target" and isinstance(body, dict):
            for triple, cfg in body.items():
                if not isinstance(cfg, dict):
                    continue
                for key in cfg:
                    if key not in TARGET_KEYS:
                        problems.append(
                            f"{rel}: `[target.{triple}] {key}` is not a key cargo "
                            f"defines there — did you mean one of "
                            f"{', '.join(sorted(TARGET_KEYS))}?"
                        )

    build_target = data.get("build", {}).get("target")
    if isinstance(build_target, str):
        configured = set(data.get("target", {}))
        if configured and build_target not in configured:
            problems.append(
                f"{rel}: `[build] target = \"{build_target}\"` names a triple this "
                f"blob does not configure ({', '.join(sorted(configured))}). "
                f"The runner and rustflags below would not apply to the build."
            )
    return problems


def scan(root):
    problems, checked = [], 0
    seen = set()
    for pattern in BOARD_GLOBS:
        for path in glob.glob(os.path.join(root, pattern)):
            if path in seen:
                continue
            seen.add(path)
            with open(path, encoding="utf-8") as fh:
                blob = blob_of(fh.read())
            if blob is None:
                continue
            checked += 1
            problems.extend(check_blob(os.path.relpath(path, root), blob))
    return problems, checked


def self_test(quiet=False):
    """Negative controls: each rule must FIRE on the shape it names."""
    with tempfile.TemporaryDirectory() as tmp:
        d = os.path.join(tmp, "packages/boards/nros-board-x")
        os.makedirs(d)
        path = os.path.join(d, "nros-board.toml")

        def write(blob):
            with open(path, "w", encoding="utf-8") as fh:
                fh.write("[[board]]\nnames = [\"x\"]\ncargo_config = '''\n%s'''\n" % blob)

        write('[target.thumbv7m-none-eabi]\nrunner = "qemu"\nrustflags = ["-C", "x"]\n')
        problems, checked = scan(tmp)
        assert checked == 1 and not problems, f"a good blob must pass; got {problems}"

        write("[target.thumbv7m-none-eabi\nrunner = 'q'\n")
        problems, _ = scan(tmp)
        assert any("not valid TOML" in p for p in problems), problems

        write('[targets.thumbv7m-none-eabi]\nrunner = "qemu"\n')
        problems, _ = scan(tmp)
        assert any("cargo does not read" in p for p in problems), problems

        # The one that matters: a typo where cargo silently ignores the key.
        write('[target.thumbv7m-none-eabi]\nrunnner = "qemu"\n')
        problems, _ = scan(tmp)
        assert any("not a key cargo" in p for p in problems), problems

        write('[build]\ntarget = "thumbv7m-none-eabi"\n[target.armv8r-none-eabihf]\nrunner = "q"\n')
        problems, _ = scan(tmp)
        assert any("does not configure" in p for p in problems), problems

    if not quiet:
        print("check-board-cargo-config-shape self-test: OK")
    return 0


def main(argv):
    if "--self-test" in argv:
        return self_test()
    rc = self_test(quiet=True)
    if rc:
        return rc

    problems, checked = scan(ROOT)
    if problems:
        print("check-board-cargo-config-shape: FAILED")
        for p in problems:
            print(f"  - {p}")
        return 1
    if checked == 0:
        print("check-board-cargo-config-shape: no `cargo_config` blob found — refusing to pass on an empty set")
        return 1
    print(f"check-board-cargo-config-shape: OK ({checked} blob(s), all well-shaped)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
