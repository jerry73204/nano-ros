/**
 * @file publisher.h
 * @ingroup grp_pubsub
 * @brief Topic publisher API.
 *
 * Create publishers with nros_publisher_init() and publish serialised
 * messages with nros_publish_raw().
 */

#ifndef NROS_PUBLISHER_H
#define NROS_PUBLISHER_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/publisher.h>` continues to compile. */
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
 * Define NROS_NO_DEPRECATED_PUBLISHER_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_PUBLISHER_ALIASES

NROS_DEPRECATED_MSG(
    "nros_publisher_get_zero_initialized() is deprecated; use rcl_get_zero_initialized_publisher()")
static inline struct nros_publisher_t nros_publisher_get_zero_initialized(void) {
    return rcl_get_zero_initialized_publisher();
}

NROS_DEPRECATED_MSG(
    "nros_publisher_get_default_options() is deprecated; use rcl_publisher_get_default_options()")
static inline struct nros_publisher_options_t nros_publisher_get_default_options(void) {
    return rcl_publisher_get_default_options();
}

NROS_DEPRECATED_MSG(
    "nros_publisher_get_topic_name() is deprecated; use rcl_publisher_get_topic_name()")
static inline const char* nros_publisher_get_topic_name(const struct nros_publisher_t* publisher) {
    return rcl_publisher_get_topic_name(publisher);
}

NROS_DEPRECATED_MSG("nros_publisher_is_valid() is deprecated; use rcl_publisher_is_valid()")
static inline bool nros_publisher_is_valid(const struct nros_publisher_t* publisher) {
    return rcl_publisher_is_valid(publisher);
}

NROS_DEPRECATED_MSG(
    "nros_publisher_assert_liveliness() is deprecated; use rcl_publisher_assert_liveliness()")
static inline nros_ret_t
nros_publisher_assert_liveliness(const struct nros_publisher_t* publisher) {
    return rcl_publisher_assert_liveliness(publisher);
}

NROS_DEPRECATED_MSG("nros_publisher_init() is deprecated; use rclc_publisher_init_default()")
static inline nros_ret_t nros_publisher_init(struct nros_publisher_t* publisher,
                                             const struct nros_node_t* node,
                                             const struct nros_message_type_t* type_info,
                                             const char* topic_name) {
    return rclc_publisher_init_default(publisher, node, type_info, topic_name);
}

#endif /* NROS_NO_DEPRECATED_PUBLISHER_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_PUBLISHER_H */
