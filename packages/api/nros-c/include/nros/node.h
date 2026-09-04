/**
 * @file node.h
 * @ingroup grp_node
 * @brief ROS 2 node creation and management.
 *
 * A node represents a participant in the ROS 2 graph.  Create one with
 * nros_node_init() after initialising a support context.
 */

#ifndef NROS_NODE_H
#define NROS_NODE_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/node.h>` continues to compile. */
#include "nros/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * DEPRECATED spellings -- phase-417 stage 6 (RFC-0089, 2026-09-04)
 *
 * RFC-0089 settled that **the C API takes rcl's spellings**: the goal is
 * drop-in replacement and a ported file's line is rcl's line. These names
 * are the pre-stage-6 `nros_` spellings, kept one release as
 * `NROS_DEPRECATED_MSG` `static inline` forwarders so an out-of-tree C node
 * that still writes them keeps compiling and gets a diagnostic naming its
 * replacement.
 *
 * An inline definition in a header has no external linkage, so every
 * translation unit may define it and none exports it: the rcl/rclc name is
 * the ONLY exported symbol. This is a SOURCE compatibility promise, not a
 * binary one -- an object file built against the pre-rename library must be
 * recompiled.
 *
 * `nros_ret_t`'s VALUES are unchanged. RFC-0089 records this as the one place
 * where taking rcl's spelling must not mean taking rcl's numbering; the
 * mapping lives in `<nros/rcl_compat.h>` and is the one part of that header
 * which does not dissolve.
 *
 * Define NROS_NO_DEPRECATED_NODE_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_NODE_ALIASES

NROS_DEPRECATED_MSG(
    "nros_node_get_zero_initialized() is deprecated; use rcl_get_zero_initialized_node()")
static inline struct nros_node_t nros_node_get_zero_initialized(void) {
    return rcl_get_zero_initialized_node();
}

NROS_DEPRECATED_MSG(
    "nros_node_get_default_options() is deprecated; use rcl_node_get_default_options()")
static inline struct nros_node_options_t nros_node_get_default_options(void) {
    return rcl_node_get_default_options();
}

NROS_DEPRECATED_MSG("nros_node_get_name() is deprecated; use rcl_node_get_name()")
static inline const char* nros_node_get_name(const struct nros_node_t* node) {
    return rcl_node_get_name(node);
}

NROS_DEPRECATED_MSG("nros_node_get_namespace() is deprecated; use rcl_node_get_namespace()")
static inline const char* nros_node_get_namespace(const struct nros_node_t* node) {
    return rcl_node_get_namespace(node);
}

NROS_DEPRECATED_MSG("nros_node_is_valid() is deprecated; use rcl_node_is_valid()")
static inline bool nros_node_is_valid(const struct nros_node_t* node) {
    return rcl_node_is_valid(node);
}

NROS_DEPRECATED_MSG("nros_node_fini() is deprecated; use rcl_node_fini()")
static inline nros_ret_t nros_node_fini(struct nros_node_t* node) {
    return rcl_node_fini(node);
}

/* The REORDER, and the reason this is a `static inline` naming each parameter
 * and never a macro: a macro would forward positionally and silently build a
 * node with its name in the support slot. Here the compiler checks each one,
 * and `const char *` against `const struct nros_support_t *` cannot be
 * confused. */
NROS_DEPRECATED_MSG("nros_node_init() is deprecated; use rclc_node_init_default(), whose "
                    "arguments are REORDERED to rclc's (node, name, namespace_, support)")
static inline nros_ret_t nros_node_init(struct nros_node_t* node,
                                        const struct nros_support_t* support, const char* name,
                                        const char* namespace_) {
    return rclc_node_init_default(node, name, namespace_, support);
}

#endif /* NROS_NO_DEPRECATED_NODE_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_NODE_H */
