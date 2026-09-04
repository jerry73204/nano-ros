#!/usr/bin/env python3
"""Decode the nano-ros boot self-report out of a target memory dump.

phase-412. The image writes a fixed record about itself into RAM
(`packages/core/nros-node/src/boot_report.rs`); this reads it back after the
core has been halted. See that module for WHY the record is not a log stream:
the failure it exists to catch halts the board during entity creation, before
any advisory can print, on a board whose console UART is not wired.

Two inputs, and the ELF is not optional: the record's address is resolved from
the image's own symbol table rather than hard-coded, so a relinked image cannot
be read at a stale address and silently decode as garbage.

    pyocd commander -t s32k344 -c halt -c "savemem ADDR LEN report.bin"
    python3 scripts/read-boot-report.py build/zephyr/zephyr.elf report.bin

`--addr-only` prints the address and length for that savemem line, so the two
steps can be scripted without anyone copying a hex number by hand:

    read $(python3 scripts/read-boot-report.py --addr-only build/zephyr/zephyr.elf)
"""

from __future__ import annotations

import argparse
import struct
import subprocess
import sys
from pathlib import Path

SYMBOL = "NROS_BOOT_REPORT"

# "NRSR". Must match boot_report.rs MAGIC.
MAGIC = 0x4E525352
# Layout this script knows how to decode. Must match boot_report.rs VERSION.
KNOWN_VERSION = 3

# Field order, matching `BootReport` and `Snapshot` in boot_report.rs. Every
# field is a u32; the record is all `AtomicU32`, which is repr(transparent).
FIELDS = (
    "magic",
    "version",
    "struct_size",
    "stage",
    "arena_size",
    "max_cbs",
    "max_sc",
    "max_nodes",
    "default_rx_buf_size",
    "arena_capacity",
    "arena_used",
    "alloc_count",
    "last_alloc_size",
    "failed_alloc_size",
    "failed_alloc_shortfall",
    "cpp_init_ret",
    "err_class",
    "err_transport",
    "err_backend_ptr",
    "err_backend_len",
)

STAGES = {
    0: "Untouched -- the image never entered nros_cpp_init",
    1: "ReportReady -- entered nros_cpp_init; arguments NOT yet validated",
    2: "BootConfigResolved -- arguments accepted; executor not yet open",
    3: "ExecutorReady -- arena bound",
    4: "RegisteringEntities (NOT YET WIRED -- no call site emits this)",
    5: "EntitiesReady (NOT YET WIRED -- registration has no single end; see issue 0900)",
    6: "FirstSpin -- registration complete and spinning",
}


# Assigned by `node_error_class` in packages/api/nros-cpp/src/lib.rs. Append
# only, and kept in step by hand -- the Rust side is exhaustive, so a new
# variant fails THERE first, which is the half that matters.
ERR_CLASS = {
    0: "(none)", 1: "Transport", 2: "NameTooLong", 3: "Serialization",
    4: "Deserialization", 5: "BufferTooSmall", 6: "ActionCreationFailed",
    7: "ServiceRequestFailed", 8: "ServiceReplyFailed", 9: "Timeout",
    10: "NotInitialized", 11: "RequestInFlight", 12: "NoSchedContextSlot",
    13: "InvalidSchedContextBinding", 14: "NodeTableFull", 15: "ExecutorFull",
    16: "BackendMismatch", 17: "ShutdownCallbacksFull",
}

# Assigned by `transport_error_class` in the same file.
ERR_TRANSPORT = {
    0: "(none)", 1: "ConnectionFailed", 2: "Disconnected",
    3: "PublisherCreationFailed", 4: "SubscriberCreationFailed",
    5: "ServiceServerCreationFailed", 6: "ServiceClientCreationFailed",
    7: "PublishFailed", 8: "ServiceRequestFailed", 9: "ServiceReplyFailed",
    10: "SerializationError", 11: "DeserializationError", 12: "BufferTooSmall",
    13: "MessageTooLarge", 14: "Timeout", 15: "InvalidConfig", 16: "WouldBlock",
    17: "TooLarge", 18: "TaskStartFailed", 19: "PollFailed",
    20: "KeepaliveFailed", 21: "JoinFailed", 22: "InvalidArgument",
    23: "Unsupported", 24: "BadAlloc", 25: "IncompatibleQos",
    26: "TopicNameInvalid", 27: "NodeNameNonExistent", 28: "LoanNotSupported",
    29: "NoData", 30: "IncompatibleAbi", 31: "Backend", 32: "BackendDynamic",
}

# Pools and tables map to NROS_CPP_RET_FULL, never to the transport code, so
# seeing any of these here rules a sizing knob IN -- and seeing a transport
# class rules every one of them OUT.
POOL_CLASSES = {5, 12, 14, 15, 17}


# --- the subscriber payload-class record (packages/rmw/zenoh/nros-rmw-zenoh) --
#
# A SECOND symbol, because the crate that owns these facts cannot call into the
# boot report: nros-node depends on nros-rmw-zenoh, so the edge runs the other
# way and a call would be a cycle. See SubscriberAllocReport's own doc comment.
ALLOC_SYMBOL = "NROS_SUBSCRIBER_ALLOC_REPORT"
ALLOC_MAGIC = 0x53554241  # "SUBA"
ALLOC_VERSION = 1
ALLOC_FIELDS = (
    "magic",
    "version",
    "struct_size",
    "refusal",
    "rx_hint",
    "largest_payload_class",
    "small_class_ceiling",
    "max_large_subscribers",
    "subscriber_large_size",
    "subscriber_buffer_size",
    "small_taken",
    "large_taken",
)
ALLOC_REFUSAL = {
    0: "none -- every subscription got a payload block",
    1: "the hint is above EVERY class, so it has nowhere legal to go",
    2: "the LARGE class is full (MAX_LARGE_SUBSCRIBERS)",
    3: "the SMALL class is full (ZPICO_MAX_SUBSCRIBERS)",
}


def resolve_symbol(elf: Path, symbol: str = SYMBOL) -> tuple[int, int]:
    """Address and size of the record, from the ELF's symbol table.

    `nm` rather than a parser, because every toolchain that produced one of
    these images ships one, and the alternative is another dependency on the
    host side of a debugging tool.
    """
    for tool in ("nm", "arm-zephyr-eabi-nm", "arm-none-eabi-nm", "llvm-nm"):
        try:
            out = subprocess.run(
                [tool, "-S", "--defined-only", str(elf)],
                capture_output=True,
                text=True,
                check=False,
            )
        except FileNotFoundError:
            continue
        if out.returncode != 0:
            continue
        for line in out.stdout.splitlines():
            parts = line.split()
            # "<addr> <size> <type> <name>" -- the size column is why -S.
            if len(parts) == 4 and parts[3] == symbol:
                return int(parts[0], 16), int(parts[1], 16)
        # The tool worked and the symbol is not there. Say so rather than
        # trying the next tool and blaming its absence.
        raise SystemExit(
            f"{elf}: no `{symbol}` symbol.\n"
            "The image was built WITHOUT the boot report. Rebuild with\n"
            "  NROS_BOOT_REPORT=1        (Zephyr: CONFIG_NROS_BOOT_REPORT=y)"
        )
    raise SystemExit("no usable `nm` found (tried nm, arm-zephyr-eabi-nm, llvm-nm)")


def decode(blob: bytes, fields: tuple[str, ...] = FIELDS) -> dict[str, int]:
    want = len(fields) * 4
    if len(blob) < want:
        raise SystemExit(
            f"dump is {len(blob)} bytes, need at least {want} for one record"
        )
    values = struct.unpack_from(f"<{len(fields)}I", blob, 0)
    return dict(zip(fields, values, strict=True))


def report(rec: dict[str, int]) -> int:
    """Print the record. Returns the process exit code."""
    if rec["magic"] != MAGIC:
        print(
            f"NO RECORD: magic is 0x{rec['magic']:08x}, expected 0x{MAGIC:08x}.\n"
            "\n"
            "The magic is written LAST by boot_report::init(), so this means the\n"
            "image did not get as far as constructing an Executor -- not that the\n"
            "dump is at the wrong address, which the ELF lookup already ruled out.\n"
            "An image that halted even earlier than that is itself the finding.",
            file=sys.stderr,
        )
        return 2

    if rec["version"] != KNOWN_VERSION:
        print(
            f"record version {rec['version']}, this script decodes {KNOWN_VERSION}.\n"
            "Refusing to decode rather than misreading it -- update this script.",
            file=sys.stderr,
        )
        return 2

    expect = len(FIELDS) * 4
    if rec["struct_size"] != expect:
        print(
            f"record says {rec['struct_size']} bytes, this script expects {expect}.\n"
            "Same version, different layout: one side changed a field without\n"
            "bumping VERSION.",
            file=sys.stderr,
        )
        return 2

    stage = rec["stage"]
    print(f"stage      {stage}  {STAGES.get(stage, 'UNKNOWN -- newer image than this script')}")
    print()
    print("compiled in (compare against what the build believed it delivered):")
    print(f"  NROS_EXECUTOR_ARENA_SIZE      {rec['arena_size']}")
    print(f"  NROS_EXECUTOR_MAX_CBS         {rec['max_cbs']}")
    print(f"  NROS_EXECUTOR_MAX_SC          {rec['max_sc']}")
    print(f"  NROS_EXECUTOR_MAX_NODES       {rec['max_nodes']}")
    print(f"  NROS_SUBSCRIPTION_BUFFER_SIZE {rec['default_rx_buf_size']}")
    print()
    print("measured on the board:")
    cap = rec["arena_capacity"]
    used = rec["arena_used"]
    print(f"  arena capacity                {cap}")
    print(f"  arena used                    {used}", end="")
    if cap:
        print(f"   ({100.0 * used / cap:.1f}%)")
    else:
        print()
    print(f"  arena allocations             {rec['alloc_count']}")
    print(f"  last allocation               {rec['last_alloc_size']} bytes")

    if cap and cap != rec["arena_size"]:
        print()
        print(
            f"  NOTE: the executor was handed {cap} bytes but the image compiled\n"
            f"  {rec['arena_size']}. Both are legitimate -- the arena's placement is the\n"
            "  caller's choice (issue 0900) -- but size against the one in use."
        )

    if rec["failed_alloc_size"]:
        print()
        print(
            f"ARENA EXHAUSTED: an allocation of {rec['failed_alloc_size']} bytes did not fit,\n"
            f"short by {rec['failed_alloc_shortfall']} bytes.\n"
            "\n"
            f"  set NROS_EXECUTOR_ARENA_SIZE >= {cap + rec['failed_alloc_shortfall']}\n"
            "  (Zephyr: CONFIG_NROS_EXECUTOR_ARENA_SIZE)\n"
            "\n"
            "That is the FIRST failure, which is the one that explains the boot;\n"
            "later allocations may also have failed as a consequence."
        )
        return 1

    cls = rec["err_class"]
    if cls:
        print()
        print(f"LAST ERROR   {ERR_CLASS.get(cls, f'unknown class {cls}')}")
        if cls == 1:
            tr = rec["err_transport"]
            print(f"  transport  {ERR_TRANSPORT.get(tr, f'unknown transport {tr}')}")
            if rec["err_backend_ptr"]:
                print(
                    f"  backend message at 0x{rec['err_backend_ptr']:08x}, "
                    f"{rec['err_backend_len']} bytes. Read it with:\n"
                    f"    pyocd commander -t <target> --connect attach \\\n"
                    f"      -c \"savemem 0x{rec['err_backend_ptr']:08x} "
                    f"{rec['err_backend_len']} msg.bin\""
                )
        if cls in POOL_CLASSES:
            print(
                "  This is a POOL or TABLE exhaustion, so a sizing knob IS the\n"
                "  cause. Raise the one the name points at."
            )
        else:
            print(
                "  NOT a pool or table exhaustion -- those map to a different\n"
                "  code. No sizing knob explains this one."
            )

    ret = rec["cpp_init_ret"]
    if ret:
        signed = ret - (1 << 32) if ret >= (1 << 31) else ret
        print()
        print(
            f"nros_cpp_init RETURNED {signed} (nros_cpp_ret_t).\n"
            "The stage above says how far it got; this says why it stopped.\n"
            "Stage 1 with a return code means an argument was rejected before\n"
            "anything was opened; stage 2 means the backend refused."
        )
        return 1

    if stage < 6:
        print()
        if rec["failed_alloc_size"]:
            pass  # already reported above
        elif rec["alloc_count"]:
            print(
                f"Never reached the first spin, and the arena is NOT why: all\n"
                f"{rec['alloc_count']} allocations succeeded and "
                f"{rec['arena_used']} of {cap} bytes are claimed.\n"
                "The executor was built and its entities took their arena; what\n"
                "did not happen is the spin. Look at what the application does\n"
                "between registering and spinning -- a blocking wait there\n"
                "presents exactly like this, and no knob is involved."
            )
        else:
            print(
                "The executor was built but claimed NO arena, so registration\n"
                "never started. That is earlier than any sizing knob can\n"
                "explain."
            )
        return 1

    return 0


def report_alloc(rec: dict[str, int]) -> int:
    """Print the subscriber payload-class record."""
    if rec["magic"] != ALLOC_MAGIC:
        print(
            f"NO ALLOC RECORD: magic is 0x{rec['magic']:08x}, expected "
            f"0x{ALLOC_MAGIC:08x}.\n"
            "alloc_payload_block was never called, so no subscription got as far\n"
            "as asking for a payload block.",
            file=sys.stderr,
        )
        return 2
    if rec["version"] != ALLOC_VERSION:
        print(f"alloc record version {rec['version']}, this script decodes "
              f"{ALLOC_VERSION}.", file=sys.stderr)
        return 2

    print("payload classes, as compiled:")
    print(f"  SUBSCRIBER_BUFFER_SIZE (small)  {rec['subscriber_buffer_size']}")
    print(f"  SMALL_CLASS_CEILING             {rec['small_class_ceiling']}")
    print(f"  SUBSCRIBER_LARGE_SIZE           {rec['subscriber_large_size']}")
    print(f"  MAX_LARGE_SUBSCRIBERS           {rec['max_large_subscribers']}")
    print(f"  LARGEST_PAYLOAD_CLASS           {rec['largest_payload_class']}")
    if rec["max_large_subscribers"] == 0:
        print(
            "  NOTE: no large slots, so the large class does not exist and the\n"
            "  ceiling is the small block. Any hint above it is refused."
        )
    print()
    print("measured on the board:")
    print(f"  small blocks taken              {rec['small_taken']}")
    print(f"  large blocks taken              {rec['large_taken']}")
    print(f"  last hint seen                  {rec['rx_hint']}")
    print()
    r = rec["refusal"]
    print(f"refusal    {r}  {ALLOC_REFUSAL.get(r, 'unknown')}")
    if r == 0:
        return 0
    print()
    if r == 1:
        print(
            f"  A subscription asked for {rec['rx_hint']} bytes and the largest class\n"
            f"  is {rec['largest_payload_class']}. Raise NROS_SUBSCRIBER_LARGE_SIZE and give the\n"
            "  image large slots (NROS_MAX_LARGE_SUBSCRIBERS), or lower the type's bound."
        )
    elif r == 2:
        print(
            f"  {rec['large_taken']} large blocks were taken and MAX_LARGE_SUBSCRIBERS is\n"
            f"  {rec['max_large_subscribers']}. Raise NROS_MAX_LARGE_SUBSCRIBERS."
        )
    else:
        print(
            f"  {rec['small_taken']} small blocks were taken. Compare that against the number\n"
            "  of subscriptions the image DECLARES: if it is higher, something\n"
            "  other than the application is taking blocks, and the derivation\n"
            "  from the entity inventory is short by that many."
        )
    return 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Decode the nano-ros boot self-report out of a target memory dump."
    )
    ap.add_argument("elf", type=Path, help="the image the board is running")
    ap.add_argument("dump", type=Path, nargs="?", help="memory dumped from the target")
    ap.add_argument(
        "--alloc",
        action="store_true",
        help="decode the subscriber payload-class record instead of the boot report",
    )
    ap.add_argument(
        "--addr-only",
        action="store_true",
        help="print '<addr> <len>' for a pyocd savemem line, and exit",
    )
    args = ap.parse_args()

    if not args.elf.is_file():
        raise SystemExit(f"{args.elf}: not a file")

    sym = ALLOC_SYMBOL if args.alloc else SYMBOL
    addr, size = resolve_symbol(args.elf, sym)
    if args.addr_only:
        print(f"0x{addr:08x} {size}")
        return 0

    if args.dump is None:
        ap.error("a dump is required unless --addr-only is given")
    if not args.dump.is_file():
        raise SystemExit(f"{args.dump}: not a file")

    blob = args.dump.read_bytes()
    if args.alloc:
        return report_alloc(decode(blob, ALLOC_FIELDS))
    return report(decode(blob))


if __name__ == "__main__":
    sys.exit(main())
