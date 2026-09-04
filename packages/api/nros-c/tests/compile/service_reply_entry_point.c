/*
 * The service reply entry point, pinned by SIGNATURE.
 *
 * Phase 379 W5 renamed the reply verb `send_reply` -> `send_response` (rcl,
 * rclcpp and rclrs all say `send_response`) and kept two old spellings one
 * release as `NROS_DEPRECATED_MSG` `static inline` forwarders:
 * `nros_service_send_reply_raw` (the plain rename) and
 * `nros_service_send_response` (a permanent NROS_RET_NOT_INIT stub from before
 * the polling API, forwarded so the un-suffixed name could not read like a
 * second, working entry point). Phase-417 stage 6 step B retired both, so what
 * this TU pins is the live name alone.
 *
 * Compile-only (no main): taking a function POINTER forces a real lookup and a
 * real signature match, so a declaration whose argument list drifts from the
 * Rust definition cbindgen emitted fails HERE rather than at some consumer's
 * call site.
 *
 * Reconstructed 2026-08-27: commit 23dcdafdc added the `just check c` lane that
 * compiles this file, but the file was not committed — so the lane referenced a
 * path that did not exist and `check-c` failed with "No such file or directory"
 * for everyone. Content follows `nros/service.h`'s own documented contract.
 */

#include "nros/service.h"

/* The live entry point, under the name the ROS client libraries use. */
static nros_ret_t (*const k_send_response_raw)(struct nros_service_t*, int64_t, const uint8_t*,
                                               size_t) = nros_service_send_response_raw;

/* Reference it so no compiler prunes the lookup this file exists to force. */
const void* nros_service_reply_entry_point_anchors[] = {
    (const void*)&k_send_response_raw,
};
