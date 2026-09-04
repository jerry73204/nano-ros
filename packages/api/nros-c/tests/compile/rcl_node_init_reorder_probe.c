/*
 * phase-417 stage 6 — the reorder probe. EXPECTED TO FAIL TO COMPILE.
 *
 * `rclc_node_init_default` is the ONE entry point in the C surface that is a
 * pure PERMUTATION of its upstream counterpart:
 *
 *     rclc  rclc_node_init_default(node, name, namespace_, support)
 *     was   nros_node_init        (node, support, name, namespace_)
 *
 * RFC-0087's corrected hazard note is why the rename and the reorder landed
 * together, and it is why this file exists:
 *
 *     C   : warning: passing argument 2 of 'f' from incompatible pointer type
 *           ...even under -Wall -Wextra.
 *     C++ : error: cannot convert 'support_t*' to 'const char*'
 *
 * So a C reorder is SILENT BY DEFAULT for exactly the callers it must not be
 * silent for: out-of-tree consumers, who do not build with our flags. Two
 * things make it loud, and both are needed:
 *
 *   1. the RENAME — a stale call under the OLD identifier `nros_node_init`
 *      reaches the deprecated forwarder, which still takes the OLD order, so
 *      it keeps working and merely warns. A caller who has moved to the NEW
 *      identifier and kept the OLD order is the dangerous case, and that is
 *      what this file writes;
 *   2. the PRAGMA — `#pragma GCC diagnostic error
 *      "-Wincompatible-pointer-types"`, scoped to this TU.
 *
 * WITHOUT THE PRAGMA THIS FILE COMPILES, and the guard proves nothing. That is
 * measured, not hypothetical: drop the `#pragma` line below and `cc
 * -fsyntax-only -std=c11 -Wall -Wextra` returns 0 on the stale call.
 *
 * Run as an expected failure:
 *
 *     cc -fsyntax-only -std=c11 -Wall -Wextra \
 *        -Itarget/nros-c-generated -Ipackages/api/nros-c/include \
 *        packages/api/nros-c/tests/compile/rcl_node_init_reorder_probe.c
 *     # must be NON-ZERO
 */

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic error "-Wincompatible-pointer-types"
#endif

#include "nros/nros.h"

nros_ret_t nros_rcl_node_init_reorder_probe(struct nros_node_t* node,
                                            const struct nros_support_t* support);
nros_ret_t nros_rcl_node_init_reorder_probe(struct nros_node_t* node,
                                            const struct nros_support_t* support) {
    /* THE STALE CALL: the pre-stage-6 argument order under the post-stage-6
     * identifier. `support` lands in the `name` slot and `"probe"` in the
     * `support` slot. Must not compile. */
    return rclc_node_init_default(node, support, "probe", "/");
}
