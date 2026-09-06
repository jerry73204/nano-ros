---
id: 1126
title: "`nros_publisher_publish_raw`'s doc promises `NROS_RET_BUFFER_TOO_SMALL`, a code `nros_ret_t` does not define"
status: open
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

## Fix

Two candidates, and they are not equivalent.

* **Correct the doc** to `NROS_RET_PUBLISH_FAILED`. Cheap, and it makes the doc
  true. It also confirms that a caller cannot distinguish "your buffer was too
  small" from "the transport refused", which for `publish_raw` may be fine —
  the caller supplied the buffer and knows its length.
* **Add `NROS_RET_FULL` to the return set** and use it, matching the FQN
  functions. Better for a caller writing generic error handling, and it is an
  ABI-visible change to what the function can return.

Prefer the first unless someone has a caller that needs to tell them apart.

## The class

A doc citing a constant from an adjacent ABI is not statically checked. Both
headers are generated and both are in scope in any TU that includes them, so
neither the compiler nor `check-c` sees a comment. A gate could reject
`NROS_RET_*` and `NROS_RMW_RET_*` spellings in doc comments that the
corresponding header does not define — worth doing only if a sweep finds more
than this one. **It does not.** `grep -rn 'NROS_RET_BUFFER_TOO_SMALL'
packages/` returns exactly two hits, and they are one site: the authored doc
comment at `packages/api/nros-c/src/publisher.rs:418` and its copy in the
generated `nros_generated.h:5554`. So this is a one-line correction plus a
header regeneration, not a class.
