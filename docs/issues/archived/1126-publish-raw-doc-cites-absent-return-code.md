---
id: 1126
title: "`nros_publisher_publish_streamed`'s doc (filed as `publish_raw`) promises `NROS_RET_BUFFER_TOO_SMALL`, a code `nros_ret_t` does not define — and four more like it"
status: resolved
resolved_in: "issue 1126 — doc corrected + class gated (check-ret-code-citations)"
type: bug
area: api
severity: low
found: 2026-09-06
related: [0785]
---

# A return code that exists on the other ABI

`packages/api/nros-c/src/publisher.rs` documents `nros_publisher_publish_raw`
as returning `NROS_RET_BUFFER_TOO_SMALL`. There is no such enumerator. The
function returns `NROS_RET_PUBLISH_FAILED`.

The constant is real, one seam over: `NROS_RMW_RET_BUFFER_TOO_SMALL`, on the
RMW ABI (`nros_rmw_ret_t`). The user ABI (`nros_ret_t`) has never had it. So a
C caller who writes the documented comparison gets a compile error naming an
undeclared identifier, with nothing pointing at the doc that told them to.

## How it was found

Writing `nros_get_fully_qualified_name` (PR #567). It needs a "your buffer is
too small" code, the doc comment three functions up named one, and the header
did not define it. That function shipped `NROS_RET_FULL` instead, and said in
its own doc why — deliberately not compounding the wrong reference.

## Fixed — the doc, and the function is not the one this issue names

**Correct-the-doc won.** `NROS_RET_FULL` (candidate 2) buys a caller nothing
here: the entry point has NO in-tree caller — `nros_publisher_publish_streamed`
appears in the tree exactly three times, in its own definition, in the generated
header, and in an archived phase doc — and the two failures it would separate
(payload over the ~4 KiB staging buffer; a `chunk_cb` that stops early) are both
the caller's own doing, from a payload the caller sized. Widening the return set
of a function nobody calls is an ABI-visible change bought for a hypothetical.

**Two corrections to the report itself**, both from reading the site: the doc
comment is on `nros_publisher_publish_streamed`, not on the raw publish (which
is spelled `nros_publish_raw` and documents its returns correctly), and the
streamed fallback reaches that error through `TransportError::BufferTooSmall`,
which `Err(_) => NROS_RET_PUBLISH_FAILED` flattens with every other backend
failure.

## The class — the sweep this issue ran was too narrow, by construction

The paragraph below said a gate was not worth it because
`grep -rn 'NROS_RET_BUFFER_TOO_SMALL' packages/` returns two hits. That grep
asks about ONE SPELLING. Asking the general question — every
`NROS_RET_*`/`NROS_RMW_RET_*` token in the tree, checked against what the two
headers actually define — finds **three live code sites and two in the book**,
of which this issue reported one:

| site | cited | reality |
| --- | --- | --- |
| `packages/api/nros-c/src/publisher.rs` | `NROS_RET_BUFFER_TOO_SMALL` | RMW-only spelling; returns `NROS_RET_PUBLISH_FAILED` |
| `packages/core/nros-rmw-abi/include/nros/rmw_vtable.h` | `NROS_RMW_RET_TRY_AGAIN` | user-ABI-only spelling; `pub_loan` reports `NROS_RMW_RET_WOULD_BLOCK` |
| `packages/core/nros-rmw/src/custom_transport.rs` | `NROS_RMW_RET_ALREADY_INIT` | defined nowhere, and no backend rejects a re-registration either |
| `book/src/porting/custom-transport.md` | `NROS_RET_ALREADY_INIT` | same claim, user-facing |
| `book/src/concepts/status-events.md` | `NROS_RMW_RET_INCOMPATIBLE_TYPE` | honestly marked "a future" code — but spelled as an identifier |

The confusion runs **both directions**, which is why guessing does not work:
`BUFFER_TOO_SMALL` exists only on the RMW ABI, `TRY_AGAIN` only on the user ABI.
And the worst of the five is in `rmw_vtable.h` — the header RFC-0054 makes the
SSoT, i.e. the document a third-party backend author writes their vtable
against.

So: **gated**, `scripts/check-ret-code-citations.py` / `just check
ret-code-citations` (fast lane, ~0.2 s, no build). It harvests the two ABIs'
vocabularies from their SSoT (`error.rs` consts, `rmw_ret.h` defines), refuses
to judge anything if that harvest looks too small, and rejects any token neither
defines. **No exemption list**: to write about a code that does not exist, drop
the prefix (`a future INCOMPATIBLE_TYPE code`) — every legitimate mention in the
tree reads better that way, and a list of spellings permitted to be wrong is the
defect wearing a badge. `archived/` is out of scope, because a historical record
should not be edited to satisfy a gate — which is why this file may keep saying
`NROS_RET_BUFFER_TOO_SMALL` below.

`check-book-identifiers.py` did not and could not catch the book pair: its
`C_IDENT` is `(nros|rcl|rclc)_[a-z0-9_]+`, lowercase by construction.

## The original argument, kept because it was wrong in an instructive way

A doc citing a constant from an adjacent ABI is not statically checked. Both
headers are generated and both are in scope in any TU that includes them, so
neither the compiler nor `check-c` sees a comment. A gate could reject
`NROS_RET_*` and `NROS_RMW_RET_*` spellings in doc comments that the
corresponding header does not define — worth doing only if a sweep finds more
than this one. **It does not.** `grep -rn 'NROS_RET_BUFFER_TOO_SMALL'
packages/` returns exactly two hits, and they are one site: the authored doc
comment at `packages/api/nros-c/src/publisher.rs:418` and its copy in the
generated `nros_generated.h:5554`.

A sweep for the SYMPTOM's spelling answers a question about that spelling. The
class is "a cited code the header does not define", and its sweep is the gate.
