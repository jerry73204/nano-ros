#!/usr/bin/env python3
"""Phase 376 W2 — our vtable against upstream's ABI, slot by slot and arg by arg.

`rmw-api-parity.py` answers "do we have the capability, somewhere". This answers
the stricter question the campaign actually set:

> Our ABI should look mostly identical to the official ABI except the RTOS
> revision. The revision can be done by adding or removing items, or fixing
> args. All RMW functions should go into the C vtable, generic over all
> backends.

So the target state is mechanical, and therefore checkable:

* every symbol in the implementation contract has a vtable slot named exactly
  the upstream name minus its `rmw_` prefix (`rmw_take` -> `take`);
* that slot's parameters match upstream's, or the difference is DECLARED with
  its RTOS reason;
* every slot with no upstream counterpart is a DECLARED addition, with a reason.

Nothing here is a matter of taste: a rename is a rename, an argument is present
or it is not. What needs judgement — whether a deviation is justified — is
exactly what the declaration tables below record, and what review looks at.

# Why "minus the prefix" rather than keeping `rmw_`

The slots live inside `nros_rmw_vtable_t`, so the type already carries the
namespace; a `rmw_` on every member would stutter. More usefully, the mechanical
rule means the comparison needs no authored name mapping at all — a mapping is a
place for a mistake to hide, and this one would have 88 entries.

Usage:
    scripts/rmw-abi-shape.py            # the report
    scripts/rmw-abi-shape.py --check    # non-zero if an undeclared deviation exists
    scripts/rmw-abi-shape.py --self-test
"""

import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VTABLE = os.path.join(ROOT, "packages", "core", "nros-rmw-abi", "include", "nros", "rmw_vtable.h")
SIGS = os.path.join(ROOT, "docs", "reference", "rmw-implementation-signatures.txt")
CONTRACT = os.path.join(ROOT, "docs", "reference", "rmw-implementation-contract.txt")

# ---------------------------------------------------------------------------
# Declared deviations. Everything here is a deliberate difference from upstream
# and needs a reason that is about the TARGET, not about our convenience.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# The vtable is a GENERIC RMW ABI, so its signatures carry no vendor name.
# `nros_rmw_publisher_t` says who wrote the header, which is the one thing a
# backend author does not need to know: the whole point of the seam is that a
# backend implements RMW, not nano-ros. The struct TAGS may stay ours — the
# typedef names are the surface.
#
# Target spelling for each type currently in a slot signature. A type with an
# upstream counterpart takes upstream's name; the RTOS-only ones take the same
# shape without a vendor prefix.
TYPE_TARGET = {
    "nros_rmw_ret_t": "rmw_ret_t",
    "nros_rmw_publisher_t": "rmw_publisher_t",
    "nros_rmw_subscription_t": "rmw_subscription_t",
    "nros_rmw_service_t": "rmw_service_t",
    "nros_rmw_client_t": "rmw_client_t",
    "nros_rmw_qos_t": "rmw_qos_profile_t",
    "nros_rmw_event_kind_t": "rmw_event_type_t",
    "nros_rmw_event_callback_t": "rmw_event_callback_t",
    "nros_rmw_publisher_options_t": "rmw_publisher_options_t",
    "nros_rmw_subscription_options_t": "rmw_subscription_options_t",
    # RTOS-only: no upstream counterpart, so the generic name is simply the
    # unprefixed one. `session` is the concept upstream splits into context and
    # node, which an image that opens exactly one of them does not need split.
    "nros_rmw_session_t": "rmw_session_t",
}

# Including our header and upstream's `rmw/rmw.h` in ONE translation unit would
# then be a redefinition. No TU in this repo does — a target image never links
# real rmw, and every host-side consumer reaches the backend through Rust — but
# it is the hazard the rename creates and it should fail loudly rather than
# produce two types of the same name. See the phase doc for the `#error` guard.
VENDOR_PREFIX = "nros_"


# Contract symbols answered by a plain exported C FUNCTION rather than a vtable
# slot, because their answer must not vary by backend.
#
# This table VERIFIES rather than records: `compare()` greps the ABI headers for
# each declaration, so an entry whose function was never declared is reported as
# missing. A table that only recorded intent would be the vacuous-test failure
# one level up — a claim of coverage with nothing behind it.
ABI_FUNCTIONS = {
    "rmw_qos_profile_check_compatible": (
        "computes over two `rmw_qos_profile_t` values with no entity, no session and "
        "no transport. A per-backend answer would be a DEFECT, and the useful call "
        "sites (create-time validation, codegen, host tooling) have no vtable to "
        "dispatch through — some run before any backend has registered"
    ),
    "rmw_compare_gids_equal": (
        "a comparison of two values this ABI defines; see qos_profile_check_compatible"
    ),
}

# One slot answering SEVERAL upstream names.
#
# The name rule is mechanical (`rmw_take` -> `take`) precisely so no authored
# mapping has to be kept true, and this table is the one deliberate exception —
# so it carries the same burden as any declared deviation: a reason, and a
# self-test proving the target slot actually exists. Without that last part an
# alias becomes a way to make a MISSING slot invisible, which is the opposite of
# what this tool is for.
GROUPED_SYMBOLS = {
    # Upstream attaches the callback to an `rmw_event_t` afterwards; ours takes
    # it at init, so `*_event_init` IS this function (phase-376 W5, A3).
    "rmw_event_set_callback": "publisher_event_init",
    # Issue 0780 — one upstream name, two slots, because upstream's
    # `rmw_event_t` carries the entity and ours does not exist.
    "rmw_take_event": "subscription_take_event",
    # Upstream split `_with_enclaves` off `rmw_get_node_names` only because
    # appending to a fixed out-parameter list would have broken its ABI. A
    # visitor has no such list, so the enclave is a fourth argument.
    #
    # The grouping is sound in SHAPE and hollow in CONTENT, and issue 0785 is
    # the record of that: nothing in this ABI accepts an enclave — no field on
    # `create_session`, none on `rmw_node_t` — so the visitor's argument is
    # structurally always NULL, and we can only ever answer "no enclave".
    # Worse than a missing slot in one specific way: the parity report counts
    # it in the ANSWERED column. It now says so, per symbol.
    #
    # And `get_node_names` is itself INERT (issue 0800): no backend fills it
    # and nothing calls it. So both names in this grouping are answered by a
    # slot that does not work yet, which is the honest state.
    "rmw_get_node_names_with_enclaves": "get_node_names",
    # Our `publish` and `take` ALREADY deal in serialized bytes — the payload
    # crossing this seam is CDR, written by `nros-serdes` above it. So these
    # three upstream names describe what our slots do; a separate slot could
    # only ever forward to the same one. The mechanical name rule attached our
    # slots to the wrong namesake, and renaming them `publish_serialized_message`
    # would be worse than recording the grouping.
    "rmw_publish_serialized_message": "publish",
    "rmw_take_serialized_message": "take",
    "rmw_take_serialized_message_with_info": "take_with_info",
    # `create_session` / `destroy_session` ARE upstream's init/shutdown, in the
    # shape an image that opens exactly one session needs. Recorded here rather
    # than left in ADDED as "no upstream equivalent", which the sibling parity
    # file contradicted by naming these very slots as our answers.
    "rmw_init": "create_session",
    "rmw_shutdown": "destroy_session",
    "rmw_context_fini": "destroy_session",
}

# Slots we add that upstream has no equivalent for.
ADDED = {
    "publisher_take_event": (
        "the publisher half of upstream's single `rmw_take_event`. Upstream "
        "needs one function because its `rmw_event_t` carries the entity; that "
        "handle is declined here, so the entity is an argument and the two "
        "sides cannot share a signature. `subscription_take_event` is the "
        "grouped answer to the upstream name; this is its twin (issue 0780)"
    ),
    # `create_session` and `destroy_session` are NOT here. They were, and their
    # own reasons said why they should not be — "grouped onto it in
    # GROUPED_SYMBOLS rather than claimed as having no upstream equivalent",
    # written directly under a comment reading "slots we add that upstream has
    # no equivalent for". They stayed only because `expected` did not union the
    # grouped targets, so removing them made both slots report as undeclared
    # extras. Fixed at the union instead of by parking them here, and the
    # self-test now refuses the overlap outright.
    "drive_io": (
        "the caller donates the thread. A backend may pump its own transport here "
        "(XRCE, single-threaded zenoh) or merely PACE the caller when it owns RX "
        "threads of its own (cyclonedds' is a sleep, and says so; uORB's is a "
        "no-op) — what the ABI never does is assume a background transport "
        "thread the target may not have. Do not shorten this back to \"the caller "
        "donates the CPU that does I/O\": that is false on half the backends"
    ),
    "next_deadline_ms": (
        "an upper BOUND on how long the backend may be left undriven before its "
        "own timers slip (lease keepalive, heartbeat, ACK-NACK); the executor "
        "caps its `drive_io` wait against it. Not \"sleep EXACTLY until the next "
        "internal event\" — its one implementation cannot do that and says so: "
        "zenoh-pico does not expose the next-keepalive timestamp through FFI, so "
        "the shim returns the constant `Z_TRANSPORT_LEASE / EXPIRE_FACTOR`. A "
        "bound is what the caller needs anyway; the value of a bound is that the "
        "loop never OVER-sleeps"
    ),
    "set_wake_callback": (
        "wake the executor from the transport's own thread or ISR, without a wait-set. "
        "SESSION-scoped: the per-entity `*_set_on_new_*_callback` slots are the upstream "
        "family, and this is not a substitute for them"
    ),
    "ping_session": (
        "reachability of the AGENT/ROUTER behind the session, which an RTOS image "
        "on a serial or agent link needs in order to reconnect: a session over "
        "such a transport can die with no entity-level symptom, and there is "
        "nothing else in this ABI that reports it (`drive_io` reports nothing on "
        "cyclonedds, whose implementation is a sleep). The old reason — "
        "\"upstream expresses liveness through a context that cannot fail\" — was "
        "a remark about upstream's data model, equally true on a workstation, "
        "which is what makes it not a target reason"
    ),
    "has_data": "a poll that allocates nothing, for a loop with no wait-set",
    "has_request": "as above, service side",
    "publish_streamed": (
        "saves the per-publisher STAGING buffer: a static `.bss` array sized for "
        "the largest message the node can send, on a target with no allocator. "
        "That is the header's own justification and it is the one that holds. It "
        "is NOT \"publish a payload larger than any single buffer the target can "
        "hold\" — neither implementation delivers that (XRCE `malloc`s the whole "
        "payload and returns MESSAGE_TOO_LARGE past one stream slot; zenoh grows "
        "a `z_owned_bytes_writer_t` to full size), and on the target class the "
        "slot exists for, XRCE's is a same-sized heap allocation in place of the "
        "static buffer — see the follow-up issue"
    ),
    "subscription_supports_in_place": (
        "the in-place capability, which slot nullity cannot carry: "
        "`RustBackendAdapter::<R>::VTABLE` is a `const` and installs "
        "`process_raw_in_place` for every `R: RustBackend`, while the answer is a "
        "runtime `&self` method because `CffiSubscription` multiplexes over "
        "whichever backend registered — zenoh true, metadata false, same nullity "
        "(issue 0781)"
    ),
    "required_rx_bytes": (
        "how many bytes of TAKE buffer a type needs, asked BEFORE the "
        "subscription exists. Upstream never asks: `rmw_take` there fills a "
        "typed `void *ros_message` the caller allocated from a heap, so the "
        "size question is answered by the typesupport above rmw. Here the "
        "payload is bytes into a caller buffer that is a `.bss` array or an "
        "arena slot, sized once at build time from a global constant, and the "
        "backend is the only party that knows what its framing and size "
        "classes add. Upstream's nearest name, "
        "`rmw_get_serialized_message_size`, is about a MESSAGE and is "
        "unimplemented in all three reference implementations (phase-403 W1)"
    ),
    "process_raw_in_place": (
        "a SCOPED borrow: it ends when the callback returns, so there is no "
        "release token to leak. Upstream's `rmw_take_loaned_message` is unscoped, "
        "and a caller who forgets the return retires one entry of a fixed-depth "
        "receive ring on a target that will never reclaim it. Not 'no copy' — "
        "upstream has a name for that and we carry it (issue 0781)"
    ),
}

# RETURN-type differences, declared. Separate from ARG_DEVIATIONS because the
# return is the channel a caller detects failure through, so a difference here
# is a different KIND of claim from a parameter difference.
RET_DEVIATIONS = {
    # The four `create_*` slots: upstream RETURNS the entity pointer and signals
    # failure with NULL; ours returns a status and writes the entity through an
    # OUT parameter. Same decision as their ARG_DEVIATIONS entry — no runtime
    # allocation, so the caller owns the storage — and returning a status is
    # strictly more informative than NULL.
    "node_get_graph_guard_condition": (
        "registers a callback and returns a status, where upstream returns the guard "
        "condition itself — see the ARG_DEVIATIONS entry"
    ),
    "create_node": "node is an OUT parameter; the return carries the status — the caller owns the node struct, as every other create here",
    "create_publisher": "entity is an OUT parameter; the return carries the status. The CALLER owns the entity struct — this is not a claim that nothing allocates (issue 0777)",
    "create_subscription": "as create_publisher",
    "create_service": "as create_publisher",
    "create_client": "as create_publisher",
    # The six `void` slots are GONE (2026-08-24). They are recorded here as a
    # deleted entry rather than an absence, because the reason they carried was
    # the shape a bad deviation takes: "cleanup is best-effort" described the
    # behaviour without justifying it, and re-reading it in W5 is what settled
    # that `void` was never a target constraint. All six now return `rmw_ret_t`
    # like upstream.
}

# Parameter differences on slots that DO correspond to an upstream function.
ARG_DEVIATIONS = {
    # ---- phase-406: correlated arguments grouped into allocator-free PODs ----
    # Each is the SAME defect as the type support: two or three arguments that
    # are meaningless apart, where upstream passes one thing. Grouping them
    # makes "pass one, forget the other" unrepresentable rather than merely
    # discouraged, and costs nothing at the ABI — every one of these structs is
    # a POD the caller builds on the stack.
    "return_loaned_message_from_publisher": (
        "upstream's loan handle is a bare `void *`; ours is an opaque "
        "`rmw_loan_token_t *`. The bare pointer let a caller return a PUBLISHER's "
        "token to a subscription, or a token from one backend to another — both "
        "compile, and both are undefined behaviour discovered at run time on a target "
        "with no allocator to notice. An incomplete type costs nothing at the ABI (it "
        "is still a pointer) and makes both a compile error. Issue 0781 asked whether "
        "these slots earn their place given nothing implements them; the answer taken "
        "in phase-406 is KEEP AND TYPE, because a slot nobody can use SAFELY is worse "
        "than one nobody uses."
    ),
    "return_loaned_message_from_subscription": (
        "as return_loaned_message_from_publisher — opaque `rmw_loan_token_t *`."
    ),
    "publish_loaned_message": (
        "as return_loaned_message_from_publisher — opaque `rmw_loan_token_t *`."
    ),
    # Keyed by slot name. Each entry is a difference from upstream's parameter
    # list that a target constraint forces, and the reason must be about the
    # TARGET rather than about our convenience.
    "take": (
        "upstream takes (sub, void *ros_message, bool *taken, allocation); ours takes "
        "(sub, buf, buf_len, size_t *out_len, bool *taken) — there is no typesupport "
        "indirection on target, so the payload is BYTES and the caller owns the buffer, "
        "which means it needs the length back; the allocation argument has nothing to "
        "pre-size — see the allocation note on `publish`"
    ),
    "take_request": (
        "upstream takes (service, rmw_service_info_t *, void *ros_request, bool *taken); ours "
        "takes bytes (buf/buf_len/out_len) because there is no typesupport on target, and a "
        "bare `int64_t *seq_out` in place of the info struct — an RTOS reply needs the sequence "
        "number and nothing else in it. NOTE issue 0778: on cyclonedds this int64 is not a "
        "sequence at all but an index into a 32-entry table released only by send_response, "
        "so a request taken and never answered leaks a slot"
    ),
    "take_response": (
        "bytes for the typed `void *ros_response`, as `take_request` — but the "
        "SECOND deviation is NOT the same one, and reading it as \"same two, "
        "client side\" is how the difference stayed invisible. `take_request` "
        "REPLACES upstream's `rmw_service_info_t *` with `*seq_out`; this slot "
        "DROPS it with nothing in its place, so a client cannot correlate a reply "
        "to a request at all. That asymmetry is the client half of issue 0778"
    ),
    # ---- Entity lifecycle ----
    # The session/node difference and the OUT-parameter shape are one decision
    # applied consistently, described in the phase doc: an image opens ONE
    # session, there is no typesupport indirection on target, and nothing is
    # allocated at create time.
    "create_node": (
        "no `rmw_context_t *`: an image has one session and reaches it directly. "
        "Node is an OUT parameter, as every other create here"
    ),
    "create_publisher": (
        "`const rmw_node_t *` matches upstream since W5/B1 — the \"session not "
        "node\" deviation is RETIRED, not declared. What remains: baked "
        "pkg/type strings plus a `type_hash` in place of upstream's "
        "`rosidl_message_type_support_t *`, because there is no typesupport "
        "indirection on target; an explicit `uint32_t domain_id`, which "
        "upstream carries in the context; and the entity as an OUT parameter, "
        "because the CALLER owns that struct's storage (upstream mallocs it and "
        "returns a pointer). That last clause is about the entity STRUCT only — "
        "the backend still heap-allocates its `backend_data` in the same call"
    ),
    "create_subscription": ("as create_publisher"),
    "create_service": ("as create_publisher"),
    "create_client": ("as create_publisher"),
    # The `const`-only deviations are GONE (2026-08-24). Fifteen slots took a
    # NON-const handle where upstream takes `const`; W5 checked every backend
    # for a write through that pointer and found none — all four `*_data_mut`
    # uses in the Rust adapter are in `destroy_*`, and no C backend touches the
    # struct — so the deviation described nothing. The entry stays as this
    # comment because "listed so it stays visible instead of settling in as
    # permanent" was the right instinct and it worked.
    # ---- The node argument, and why it is absent ----
    #
    # "An image has no node object" was TRUE when W3.c wrote it and stopped
    # being true on 2026-08-24, when W4 landed `rmw_node_t` plus `create_node` /
    # `destroy_node`. Twelve entries still argued from it. The conclusions
    # survive, on two DIFFERENT better reasons:
    #   - teardown and availability: upstream's node argument is VALIDATION
    #     context; our entity's `backend_data` is self-sufficient, so the
    #     parameter would be inert.
    #   - counts and the graph family: these are SESSION-wide facts, and our
    #     `rmw_node_t` deliberately carries no `context` field, so a backend
    #     handed only a node could not reach the session to answer. Upstream
    #     reaches its context THROUGH the node; we pass the session directly.
    "destroy_publisher": (
        "upstream takes (node, entity). The node is VALIDATION context there; here "
        "the entity's `backend_data` is self-sufficient, so the parameter would be "
        "inert. NOT \"an image has no node object\" — it has had one since W4"
    ),
    "destroy_subscription": ("as destroy_publisher"),
    "destroy_service": ("as destroy_publisher"),
    "destroy_client": ("as destroy_publisher"),
    # ---- Data plane ----
    "publish": (
        "bytes (`const uint8_t *`, `size_t`) rather than a typed `const void *`, because "
        "codegen bakes the type and there is no typesupport on target; and no "
        "allocation argument, because upstream's allocation argument is an OPAQUE per-implementation handle (`rmw_publisher_allocation_t` is `{const char *implementation_identifier; void *data;}` — verified against Humble's `rmw/types.h`, and it contains no allocator), produced only by `rmw_init_publisher_allocation`, whose OTHER two parameters are a typesupport pointer and a `rosidl_runtime_c__Sequence__bound *` — both declined ABI-wide. With no way to produce one, the argument has nothing to point at. "
        "TWO reasons have been wrong here in a week, both of them plausible: "
        "'pools are baked' (FALSE — issue 0777 measured cyclonedds calling "
        "ddsrt_malloc/calloc per publish AND per take, zenoh inside zenoh-pico, "
        "the cffi shim per fallback loan; only uORB preallocates), then 'upstream "
        "pre-sizes an rcutils_allocator_t the caller owns' (FALSE — there is no "
        "allocator in that struct). Check the struct before writing a third"
    ),
    "publish_loaned_message": ("a length instead of upstream's allocation argument: the loan is a byte slot, and the backend needs to know how much of it was written"),
    "borrow_loaned_message": ("upstream loans a typed message via `void **`; ours reserves a byte slot of a requested size and reports the granted capacity plus an opaque token, because the payload is bytes and the backend owns the buffer until it is committed or discarded"),
    "subscription_take_event": (
        "no `rmw_event_t *` — that handle is declined, so the entity plus the "
        "`kind` identifies the event — and the payload is the typed "
        "`rmw_event_payload_t` union rather than upstream's `void *`, whose "
        "shape the caller has to know out of band"
    ),
    # ---- Content filter / network flows: allocation replaced by a visitor ----
    "subscription_get_network_flow_endpoints": ("as publisher_get_network_flow_endpoints"),
    # ---- Events ----
    "publisher_event_init": (
        "upstream fills an `rmw_event_t` the caller then polls with "
        "`rmw_take_event`; ours registers a CALLBACK directly, because an RTOS "
        "executor has no wait-set to poll an event handle from. Taking the "
        "callback at init also FUSES upstream's `rmw_event_set_callback` into "
        "this slot, at the cost of being unable to replace or clear one later. "
        "The extra `uint32_t` is `deadline_ms`, a duration consulted for the "
        "DEADLINE_MISSED kinds only — it read \"the QoS-policy filter\" until "
        "2026-08-24, which is a different thing that does not exist here — and "
        "`void *` is the callback context"
    ),
    "subscription_event_init": ("as publisher_event_init"),
    "get_node_names": (
        "visitor instead of an allocating `rcutils_string_array_t` pair; session "
        "not node; and the `enclave` argument is what lets ONE slot answer both "
        "`rmw_get_node_names` and `rmw_get_node_names_with_enclaves` — upstream "
        "split those only because appending to a fixed out-parameter list would "
        "have broken its ABI, which a visitor has no equivalent of"
    ),
    "get_topic_names_and_types": ('upstream returns an ALLOCATING `rmw_names_and_types_t` / `rcutils_string_array_t`; ours streams through a visitor callback, because there is no allocator at this seam and the ROS graph has no bound the CALLER can know — a buffer shape would make a 128 KiB image reserve for the worst graph it might ever meet. Session, not node, as everywhere else'),
    "get_service_names_and_types": ('upstream returns an ALLOCATING `rmw_names_and_types_t` / `rcutils_string_array_t`; ours streams through a visitor callback, because there is no allocator at this seam and the ROS graph has no bound the CALLER can know — a buffer shape would make a 128 KiB image reserve for the worst graph it might ever meet. Session, not node, as everywhere else'),
    "get_publisher_names_and_types_by_node": ('upstream returns an ALLOCATING `rmw_names_and_types_t` / `rcutils_string_array_t`; ours streams through a visitor callback, because there is no allocator at this seam and the ROS graph has no bound the CALLER can know — a buffer shape would make a 128 KiB image reserve for the worst graph it might ever meet. Session, not node, as everywhere else'),
    "get_subscriber_names_and_types_by_node": ('upstream returns an ALLOCATING `rmw_names_and_types_t` / `rcutils_string_array_t`; ours streams through a visitor callback, because there is no allocator at this seam and the ROS graph has no bound the CALLER can know — a buffer shape would make a 128 KiB image reserve for the worst graph it might ever meet. Session, not node, as everywhere else'),
    "get_service_names_and_types_by_node": ('upstream returns an ALLOCATING `rmw_names_and_types_t` / `rcutils_string_array_t`; ours streams through a visitor callback, because there is no allocator at this seam and the ROS graph has no bound the CALLER can know — a buffer shape would make a 128 KiB image reserve for the worst graph it might ever meet. Session, not node, as everywhere else'),
    "get_client_names_and_types_by_node": ('upstream returns an ALLOCATING `rmw_names_and_types_t` / `rcutils_string_array_t`; ours streams through a visitor callback, because there is no allocator at this seam and the ROS graph has no bound the CALLER can know — a buffer shape would make a 128 KiB image reserve for the worst graph it might ever meet. Session, not node, as everywhere else'),
    "get_publishers_info_by_topic": ('upstream returns an ALLOCATING `rmw_topic_endpoint_info_array_t` — NOT `rmw_names_and_types_t`, which is the sibling family this entry was copy-pasted from; ours streams through a visitor callback, because there is no allocator at this seam and the ROS graph has no bound the CALLER can know — a buffer shape would make a 128 KiB image reserve for the worst graph it might ever meet. Session, not node, as everywhere else'),
    "get_subscriptions_info_by_topic": ('upstream returns an ALLOCATING `rmw_topic_endpoint_info_array_t` — NOT `rmw_names_and_types_t`, which is the sibling family this entry was copy-pasted from; ours streams through a visitor callback, because there is no allocator at this seam and the ROS graph has no bound the CALLER can know — a buffer shape would make a 128 KiB image reserve for the worst graph it might ever meet. Session, not node, as everywhere else'),
    "count_publishers": (
        "session, not node: a matched count is a SESSION-wide fact and our "
        "`rmw_node_t` carries no `context`, so a backend handed only a node could "
        "not reach the session to answer it. Upstream reaches its context THROUGH "
        "the node. NOT \"an image has no node object\""
    ),
    "count_subscribers": ("as count_publishers"),
    "node_get_graph_guard_condition": (
        "`rmw_session_t *` for upstream's `const rmw_node_t *`, as the rest of the "
        "graph family and for the same reason. And upstream RETURNS a guard "
        "condition for the caller to add to a wait set; we "
        "have no wait set and guard conditions are an executor concept here, so this "
        "registers a callback instead — the `set_wake_callback` shape. The callback "
        "is an EDGE with no payload: saying WHAT changed means buffering it, which "
        "is the graph cache a small target cannot afford"
    ),
    "take_with_info": (
        "the same two deviations `take` declares — bytes rather than a typed "
        "`void *ros_message` because there is no typesupport on target, and no "
        "allocation argument — see `publish`"
    ),
    "take_loaned_message_with_info": (
        "the same deviations `take_loaned_message` declares — a byte view plus an "
        "opaque release token instead of a typed loan"
    ),
    "publisher_wait_for_all_acked": (
        "`uint32_t timeout_ms` for upstream's by-value `rmw_time_t {int64 sec, "
        "uint64 nsec}`: 4 bytes against 16 in every entity and every call, and no "
        "64-bit division on a 32-bit MCU. Millisecond resolution and a ~49.7-day "
        "ceiling are the price (issue 0241). Do NOT restore the old claim that "
        "\"every duration in this ABI is u32\" — `drive_io` and `ping_session` take "
        "`int32_t timeout_ms`, so it is one unit and TWO widths"
    ),
    "send_request": (
        "bytes rather than a typed `const void *`, and no `int64_t *sequence_id` "
        "OUT parameter: upstream hands the caller the sequence it assigned, while "
        "ours is a fire-and-forget publish whose reply is matched by "
        "`take_response`. W5 verdict: this is a GAP, not a deviation — filed as issue "
        "0778. Nothing matches the reply, because there is nothing to match it BY, and "
        "each backend invented an unsafe policy to cope (cyclone abandons the first "
        "request, zenoh assumes idempotence)"
    ),
    "send_response": (
        "bytes plus a bare `int64_t` sequence rather than upstream's "
        "`rmw_request_id_t *`: an RTOS reply needs the sequence and nothing else "
        "in that struct, the same deviation `take_request` declares on the way in"
    ),
    "take_sequence": (
        "upstream takes (sub, count, message_sequence, info_sequence, size_t *taken, allocation); "
        "ours takes a contiguous byte block plus a per-slot length array, because there is no "
        "typed message sequence on target and no allocator to pre-size"
    ),
    "take_loaned_message": (
        "upstream loans a typed `void **loaned_message`; ours is a byte view plus an opaque "
        "token to release, because there is no typesupport and the backend owns the buffer "
        "until `sub_release`"
    ),
    "service_server_is_available": (
        "upstream takes (node, client, bool *); the node is validation context "
        "there and the client reaches its session directly here, so the parameter "
        "would be inert. NOT \"an image has no node object\""
    ),
}

# Contract symbols we deliberately do not implement at all.
from importlib import util as _util  # noqa: E402  (kept beside its use)

_spec = _util.spec_from_file_location("_parity", os.path.join(ROOT, "scripts", "rmw-api-parity.py"))
_parity = _util.module_from_spec(_spec)
_spec.loader.exec_module(_parity)
DECLINED = {k for k, (bucket, _why) in _parity.MAP.items() if bucket == "declined"}

# `layer` — answered, but not through the vtable and not as a C ABI function
# either: `rmw_serialize` / `rmw_deserialize` are nros-serdes plus the codegen
# packs. Skipped like `declined`, and counted SEPARATELY so the report never
# implies they are missing. (The two `layer` symbols that ARE C functions in
# our headers are in ABI_FUNCTIONS and verified there instead.)
LAYERED = {
    k for k, (bucket, _why) in _parity.MAP.items() if bucket == "layer"
}

# A `gap` whose reason names a TRACKED ISSUE is deferred, not forgotten, and
# `--check` must not treat it as a red — otherwise the only way to put this
# gate on the `just check` line is to stop tracking the gap, which is exactly
# backwards. The issue id is the whole requirement: a bare "not yet" would let
# anything sit here, while `issue 0776` is a file somebody has to close.
_ISSUE_REF = re.compile(r"\bissue[ -]?(\d{4})\b", re.I)
DEFERRED = {
    k: _ISSUE_REF.search(why).group(1)
    for k, (bucket, why) in _parity.MAP.items()
    if bucket == "gap" and _ISSUE_REF.search(why)
}


def vtable_slots():
    """{slot: [param types]} from the vtable header.

    Comments are stripped first: the header documents each slot in prose that
    names other slots, and a prose mention must not read as a declaration
    (issue 0719's trap).
    """
    src = open(VTABLE, encoding="utf-8").read()
    body = re.sub(r"/\*.*?\*/", " ", src, flags=re.S)
    body = re.sub(r"(?m)//.*$", " ", body)

    # Only the vtable STRUCT's members are slots. Phase 376 W4 added
    # file-scope visitor typedefs (`rmw_node_visit_fn` and friends) which match
    # the same `(*name)(` shape, and scanning the whole file reported all three
    # as undeclared extra slots — a tool defect that reads as a finding.
    body = body[body.index("typedef struct nros_rmw_vtable_t {"):]
    body = body[: body.index("} nros_rmw_vtable_t;")]

    slots = {}
    rets = {}
    # `[*\s]*` after the type name: a slot may RETURN a pointer
    # (`const char *(*get_implementation_identifier)(void)`). Without it the
    # regex silently skipped every pointer-returning slot, so two slots that had
    # already landed were reported as MISSING — a tool defect that reads as a
    # gap in the ABI.
    for m in re.finditer(r"([A-Za-z_][A-Za-z0-9_ ]*?[\w])[*\s]*\(\s*\*\s*([a-z_0-9]+)\s*\)\s*\(", body):
        # Keep the pointer in the return type: the capture group deliberately
        # stops at the last word so `const char *(*slot)(…)` parses at all, so
        # the `*`s have to be put back or every pointer-returning slot reports a
        # spurious `const char` vs `const char *` difference.
        # Between the type's last word and the opening `(` of `(*slot)` — the
        # slot's OWN `*` lives after that paren and must not be counted.
        gap = m.group(0)[m.end(1) - m.start(0) :]
        gap = gap[: gap.index("(")]
        ret = " ".join((m.group(1) + " " + "*" * gap.count("*")).split())
        name = m.group(2)
        depth = 1
        params = ""
        for ch in body[m.end():]:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    break
            params += ch
        # A nested function-pointer parameter (`set_wake_callback`'s `cb`) is
        # matched by the same regex; it is a parameter, not a slot.
        slots.setdefault(name, _norm(params))
        rets.setdefault(name, ret)
    return slots, rets


def _norm(raw):
    """Same normalisation the inventory applies, so both sides are comparable.

    `normalise_params` KEEPS parameter names now (they are dropped by the
    comparison, not by the record), so this strips them again: this function is
    the comparison side, and it exists precisely so both sides are the same
    shape. Missing that coupling turned every argument into a "dropped" one —
    53 slots at once — because our side carried names and the upstream extract
    did not.
    """
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    spec = _util.spec_from_file_location("_inv", os.path.join(ROOT, "scripts", "rmw-api-inventory.py"))
    mod = _util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return strip_param_names(mod.normalise_params(raw))


def strip_param_names(params):
    """`["rmw_node_t * node", "void (*cb)(void *)"]` -> types only.

    THE one place a parameter name is dropped. The extract keeps names because
    the rendered page shows them beside our own named slots; the COMPARISON
    drops them because a renamed argument is not an ABI difference and
    reporting one trains people to skim (issue: the rationale that used to sit
    on `rmw-api-inventory.normalise_params`, moved to the layer it is actually
    about).
    """
    out = []
    for p in params:
        p = " ".join(p.split())
        if not p:
            continue
        # A function-pointer parameter carries its name inside the parens.
        if "(*" in p:
            out.append(re.sub(r"\(\*\s*[A-Za-z_][A-Za-z0-9_]*\s*\)", "(*)", p))
            continue
        m = re.match(r"^(.*?[\s*])([A-Za-z_][A-Za-z0-9_]*)((\s*\[\s*\])?)$", p)
        if m:
            p = (m.group(1) + m.group(3)).strip()
        out.append(" ".join(p.split()))
    return out


def upstream_signatures(with_names=False):
    """{name: [param types]} for the implementation contract only."""
    if not os.path.exists(SIGS) or not os.path.exists(CONTRACT):
        return None
    contract = {
        l.strip() for l in open(CONTRACT, encoding="utf-8") if l.strip() and not l.startswith("#")
    }
    # Does this extract carry parameter names? Format 1 dropped them at
    # extraction; format 2 keeps them and leaves stripping to the comparison.
    # Stripping a format-1 line AGAIN removes the last token of a by-value
    # parameter, so the marker is load-bearing rather than decorative.
    text = open(SIGS, encoding="utf-8").read()
    has_names = "# format: 2" in text
    sigs = {}
    for line in text.splitlines(keepends=True):
        if line.startswith("#") or not line.strip():
            continue
        parts = line.rstrip("\n").split("\t")
        if len(parts) < 3:
            continue
        name, ret, params = parts[0], parts[1], parts[2]
        if name in contract:
            parsed = [p.strip() for p in params.split(",") if p.strip()]
            if has_names and not with_names:
                parsed = strip_param_names(parsed)
            sigs[name] = (" ".join(ret.split()), parsed)
    return sigs


def compare():
    slots, rets = vtable_slots()
    up = upstream_signatures()
    if up is None:
        return None

    # Callback PARAMETERS caught by the slot regex are not slots.
    for probably_a_param in ("cb", "chunk_cb", "size_cb"):
        slots.pop(probably_a_param, None)

    missing, arg_diff, matched, declared, ret_diff, grouped, abi_fns = (
        [], [], [], [], [], [], [])
    deferred = []
    layered = []

    # What the headers actually DECLARE, so the table above cannot claim a
    # function nobody wrote.
    hdr = ""
    inc = os.path.join(ROOT, "packages", "core", "nros-rmw-abi", "include", "nros")
    for fn in sorted(os.listdir(inc)):
        if fn.endswith(".h"):
            hdr += open(os.path.join(inc, fn), encoding="utf-8").read()
    hdr = re.sub(r"/\*.*?\*/", " ", hdr, flags=re.S)
    declared_fns = {
        n for n in ABI_FUNCTIONS if re.search(r"\b%s\s*\(" % re.escape(n), hdr)
    }
    for name, (up_ret, params) in sorted(up.items()):
        if name in DECLINED:
            continue
        if name in LAYERED and name not in ABI_FUNCTIONS:
            layered.append(name)
            continue
        if name in ABI_FUNCTIONS:
            if name in declared_fns:
                abi_fns.append(name)
            else:
                missing.append((name, name, params))
            continue
        slot = GROUPED_SYMBOLS.get(name) or name[len("rmw_"):]
        if name in GROUPED_SYMBOLS:
            # An alias is satisfied by its target existing; the target's own
            # entry is what checks the signature.
            if slot in slots:
                grouped.append((name, slot))
                continue
        if slot not in slots:
            (deferred if name in DEFERRED else missing).append((name, slot, params))
            continue
        # Phase 376 W5 — the RETURN type is part of a signature. This tool did
        # not parse it, so six slots returned `void` where upstream returns
        # `rmw_ret_t` — a difference nobody had declared, on the axis that
        # decides whether a caller can detect failure at all — and `--check` was
        # about to join `just check` blind to it.
        ret_ok = rets.get(slot, "") == up_ret
        if slots[slot] == params and ret_ok:
            matched.append(slot)
        elif slot in ARG_DEVIATIONS and (ret_ok or slot in RET_DEVIATIONS):
            declared.append(slot)
        elif not ret_ok and slot not in RET_DEVIATIONS:
            ret_diff.append((slot, up_ret, rets.get(slot, "?")))
        else:
            arg_diff.append((slot, params, slots[slot]))

    # Grouped TARGETS are expected too. `create_session` / `destroy_session`
    # have no upstream name of their own (they answer `rmw_init` and
    # `rmw_shutdown` + `rmw_context_fini`), so without this they read as
    # undeclared extras — which is why both were parked in `ADDED`, a table
    # whose comment says "upstream has no equivalent" while their own reasons
    # say the opposite. The headline then claimed 11 RTOS additions when 9 are
    # additions.
    expected = {n[len("rmw_"):] for n in up} | set(ADDED) | set(GROUPED_SYMBOLS.values())
    undeclared_extra = sorted(s for s in slots if s not in expected)

    # Vendor-named types in the signatures — the "generic flavour" rule.
    vendor_types = {}
    for slot, params in slots.items():
        for p in params:
            for tok in re.findall(r"\b[A-Za-z_][A-Za-z0-9_]*\b", p):
                if tok.startswith(VENDOR_PREFIX):
                    vendor_types.setdefault(tok, set()).add(slot)

    return {
        "vendor_types": vendor_types,
        "slots": slots,
        "upstream": up,
        "missing": missing,
        "deferred": deferred,
        "layered": layered,
        "arg_diff": arg_diff,
        "matched": matched,
        "abi_fns": abi_fns,
        "grouped": grouped,
        "declared": declared,
        "ret_diff": ret_diff,
        "undeclared_extra": undeclared_extra,
    }


def self_test():
    bad = []
    slots, _rets = vtable_slots()
    if "create_publisher" not in slots:
        bad.append("vtable parse found no create_publisher slot")
    if "cb" in slots and len(slots.get("cb", [])) == 0:
        pass  # popped in compare(); parsing it here is fine
    # A prose mention must not become a slot.
    probe = re.sub(r"/\*.*?\*/", " ", "/* calls (*take)(x) internally */ int (*real)(int a);", flags=re.S)
    names = re.findall(r"\(\s*\*\s*([a-z_0-9]+)\s*\)\s*\(", probe)
    if names != ["real"]:
        bad.append(f"a slot named only in a COMMENT must not count: {names}")
    r = compare()
    if r is not None:
        for tname in r["vendor_types"]:
            if tname not in TYPE_TARGET:
                bad.append(f"{tname}: in a signature with no target spelling in TYPE_TARGET")
    slots_now, _r = vtable_slots()
    for sym, target in GROUPED_SYMBOLS.items():
        if target not in slots_now:
            bad.append(
                f"{sym}: grouped onto {target!r}, which is NOT a slot — an alias to a "
                "missing slot hides the gap it should report"
            )
    both = set(ADDED) & set(GROUPED_SYMBOLS.values())
    if both:
        bad.append(
            f"slot(s) in ADDED *and* a GROUPED target: {sorted(both)} — a slot "
            "cannot both have no upstream equivalent and be the answer to an "
            "upstream name"
        )
    if not ADDED:
        bad.append("ADDED is empty — every RTOS-only slot must carry its reason")
    for slot, why in ADDED.items():
        if not why.strip():
            bad.append(f"{slot}: an addition with no reason is prose")
    # The deferral rule, both ways: a `gap` reason without an issue id is NOT
    # deferred, it is a red. Without this the rule degrades into "any gap is
    # fine", which is the failure mode that would make `--check` on the
    # `just check` line worth nothing.
    for probe, want in (("issue 0776. the bound is not computed", True),
                        ("issue-0776 tracked", True),
                        ("not yet, no bound is computed", False),
                        ("tracked in issue 77", False)):
        if bool(_ISSUE_REF.search(probe)) != want:
            bad.append(f"issue-ref rule: {probe!r} should be deferred={want}")
    # The DERIVATION, not the tree's current contents. This asserted
    # `if not DEFERRED` until 2026-08-27, which conflated "the link to the
    # parity MAP is broken" with "there is nothing to derive" — and the second
    # became true the moment the last gap closed (issue 0776 -> `layer`, once
    # phase-380 W1 landed the size bound). A tripwire that fires when the tree
    # gets HEALTHIER is a tripwire people learn to route around, so it is now
    # checked against a synthetic mapping and holds whether or not any real gap
    # exists.
    _probe_map = {
        "rmw_synthetic_deferred": ("gap", "issue 0776. deliberately unresolved"),
        "rmw_synthetic_plain_gap": ("gap", "no issue named, so not deferred"),
        "rmw_synthetic_answered": ("vtable", "issue 0776 mentioned but not a gap"),
    }
    derived = {
        k: _ISSUE_REF.search(why).group(1)
        for k, (bucket, why) in _probe_map.items()
        if bucket == "gap" and _ISSUE_REF.search(why)
    }
    if derived != {"rmw_synthetic_deferred": "0776"}:
        bad.append(f"deferred derivation: got {derived}, want only the issue-bearing gap")

    # A deviation entry for a slot that no longer deviates is the same drift
    # the parity MAP had (45 stale entries, two directions). Three ARG entries
    # and six RET entries survived their own fix here, still explaining a
    # difference the header had stopped having — and the RET ones read
    # "NOT a target constraint, W5 candidate to change", which is a to-do
    # item that had been done. Declarations are only worth reading if a stale
    # one is impossible.
    up = upstream_signatures()
    if up is not None:
        our_args, our_rets = vtable_slots()
        # A handle upstream takes as `const` must be `const` here too.
        #
        # Matched BY TYPE, never by position — the first sweep of this class
        # (15 slots, commit 0814b645d) compared arguments positionally and so
        # missed three: upstream puts an `rmw_event_t *` or a `const rmw_node_t *`
        # ahead of the handle in `{publisher,subscription}_event_init` and
        # `service_server_is_available`, and ours does not, so zip() lined the
        # handle up against something else and saw nothing. The class fix
        # stopped one short of the class for a reason that had nothing to do
        # with the class.
        handles = {
            "rmw_publisher_t", "rmw_subscription_t", "rmw_service_t",
            "rmw_client_t", "rmw_node_t", "rmw_session_t",
        }

        def _handle_constness(params):
            out = {}
            for prm in params:
                m = re.match(r"^(const )?(rmw_\w+_t) \*$", prm)
                if m and m.group(2) in handles:
                    out[m.group(2)] = bool(m.group(1))
            return out

        for name, (_uret, uargs) in sorted(up.items()):
            slot = GROUPED_SYMBOLS.get(name) or name[len("rmw_"):]
            if slot not in our_args:
                continue
            theirs = _handle_constness(uargs)
            ours = _handle_constness(our_args[slot])
            for ty, is_const in theirs.items():
                if is_const and ty in ours and not ours[ty]:
                    bad.append(
                        f"{slot}: takes a non-const `{ty} *` where upstream takes "
                        "`const` — no backend writes the handle, so this is a "
                        "signature to fix, not a deviation to declare"
                    )
        # Per SLOT, not per bucket: a slot can legitimately hold an ARG entry
        # and a stale RET entry at once (`destroy_publisher` did — it still
        # drops upstream's node argument, which kept it in the "declared"
        # bucket and hid that its return had stopped differing). So compare
        # each axis against the header directly.
        up_by_slot = {}
        for name, (uret, uargs) in up.items():
            up_by_slot[GROUPED_SYMBOLS.get(name) or name[len("rmw_"):]] = (uret, uargs)
        for s in sorted(ARG_DEVIATIONS):
            u = up_by_slot.get(s)
            if u and s in our_args and our_args[s] == u[1]:
                bad.append(
                    f"ARG_DEVIATIONS[{s!r}] explains an argument difference the header "
                    "no longer has"
                )
        for s in sorted(RET_DEVIATIONS):
            u = up_by_slot.get(s)
            if u and s in our_rets and our_rets[s] == u[0]:
                bad.append(
                    f"RET_DEVIATIONS[{s!r}] explains a return-type difference the header "
                    "no longer has"
                )

    if bad:
        for b in bad:
            sys.stderr.write("rmw-abi-shape --self-test: " + b + "\n")
        return 2
    print(f"rmw-abi-shape --self-test: OK ({len(slots)} slot(s) parsed, 7 case(s))")
    return 0


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()

    r = compare()
    if r is None:
        sys.stderr.write(
            f"rmw-abi-shape: missing {SIGS} or {CONTRACT}.\n"
            "  Regenerate in the distrobox:\n"
            "    scripts/rmw-api-inventory.py --signatures > docs/reference/rmw-implementation-signatures.txt\n"
        )
        return 2

    total = (len(r["upstream"]) - len(DECLINED & set(r["upstream"]))
             - len(r["layered"]))
    print("rmw ABI shape — our vtable against upstream, name and args")
    print(f"  contract symbols to mirror : {total}")
    # Phase 376 W5 — three numbers, not two. This used to print ONE, counting a
    # slot with a DECLARED deviation as "matching name + args", so the headline
    # said 24 exact matches when 20 of those were declared differences. A
    # measurement that folds "identical" into "different but explained" cannot
    # answer the question the campaign is actually asking.
    print(f"  slots identical to upstream : {len(r['matched'])}")
    print(f"  name matches, args DECLARED : {len(r['declared'])}")
    print(f"  answered by a GROUPED slot  : {len(r['grouped'])}")
    print(f"  plain ABI functions         : {len(r['abi_fns'])}")
    print(f"  answered at another layer   : {len(r['layered'])}")
    print(f"  slots present, args differ : {len(r['arg_diff'])}")
    print(f"  UNDECLARED return-type diff: {len(r['ret_diff'])}")
    print(f"  no slot at all             : {len(r['missing'])}")
    print(f"  deferred, issue tracked    : {len(r['deferred'])}")
    print(f"  declared RTOS additions    : {len(ADDED)}")
    print(f"  UNDECLARED extra slots     : {len(r['undeclared_extra'])}")
    print(f"  vendor-named types in sigs : {len(r['vendor_types'])}")
    print()

    if r["vendor_types"]:
        print(f"## vendor-named types in slot signatures ({len(r['vendor_types'])})")
        print("   the vtable is a generic RMW ABI; a backend implements RMW, not nano-ros")
        for tname in sorted(r["vendor_types"]):
            target = TYPE_TARGET.get(tname, "?? no target spelling recorded")
            uses = len(r["vendor_types"][tname])
            print(f"  {tname:34s} -> {target:30s} ({uses} slot(s))")
        print()

    if r["ret_diff"]:
        print(f"## return type differs, undeclared ({len(r['ret_diff'])})")
        for slot, want, got in r["ret_diff"]:
            print(f"  {slot:44s} upstream {want!r}, ours {got!r}")
        print()

    if r["arg_diff"]:
        print(f"## args differ ({len(r['arg_diff'])})")
        for slot, want, got in r["arg_diff"]:
            print(f"  {slot}")
            print(f"      upstream: ({', '.join(want)})")
            print(f"      ours:     ({', '.join(got)})")
        print()

    if r["undeclared_extra"]:
        print(f"## extra slots with no declaration ({len(r['undeclared_extra'])})")
        for s in r["undeclared_extra"]:
            print(f"  {s}")
        print()

    if r["deferred"]:
        print(f"## deferred, tracked by an issue ({len(r['deferred'])})")
        for name, slot, params in r["deferred"]:
            print(f"  {slot:44s} <- issue {DEFERRED[name]}")
        print()

    if r["missing"]:
        print(f"## no slot ({len(r['missing'])})")
        for name, slot, params in r["missing"]:
            print(f"  {slot:44s} <- {name}({', '.join(params)})")
        print()

    rc = 0
    if args.check and (
        r["missing"] or r["arg_diff"] or r["ret_diff"] or r["undeclared_extra"]
        or r["vendor_types"]
    ):
        sys.stderr.write(
            "rmw-abi-shape: the vtable does not mirror upstream.\n"
            "  Every contract symbol needs a slot named after it, with matching\n"
            "  args — or an entry in ARG_DEVIATIONS / ADDED / the parity table's\n"
            "  `declined` bucket, carrying the RTOS reason for the difference.\n"
            "  A gap that is real but not yet closed goes in the parity table's\n"
            "  `gap` bucket with a TRACKED ISSUE ID in the reason (`issue 0776`),\n"
            "  which is deferral with a name on it rather than an exemption.\n"
            "  Signatures must also be vendor-free: see TYPE_TARGET.\n"
        )
        rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
