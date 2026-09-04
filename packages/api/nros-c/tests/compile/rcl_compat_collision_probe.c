/*
 * Phase-417 W5.b — the NEGATIVE half of `rcl_compat_aliases.c`.
 *
 * That file proves the rcl spellings RESOLVE. This one proves the header
 * REFUSES to sit in a translation unit that already has the real rcl.
 *
 * Why it has to be a compile failure and not a runtime check: nano-ros and rcl
 * define the same `RCL_RET_*` names with DIFFERENT VALUES -- ours are
 * -1/-2/-3/-7, rcl's are 1/2/11/101 (`rcl/types.h:26-41`,
 * `rmw/ret_types.h:29-38`). Whichever header won the include race would supply
 * the constants, the other's functions would return the other numbering, and
 * every comparison in the file would compile and be wrong. There is no value a
 * program could inspect afterwards to tell which happened. That is exactly the
 * "compiles and differs" RFC-0089 forbids, so the collision must be fatal at
 * the earliest point it is KNOWABLE, which is the preprocessor.
 *
 * `RCL_RET_OK` stands in for the whole real header here. Defining it is what
 * `#include <rcl/types.h>` does, and this repo has no rcl to include -- it is
 * not a dependency, and the parity tooling reads rcl from an out-of-tree
 * checkout for the same reason. Using the sentinel keeps the probe runnable on
 * a host with no ROS, which is the condition every check in this lane holds to.
 *
 * `just check c` compiles this and requires it to FAIL. Written as an expected
 * failure because a clean compile is exactly what a missing guard looks like.
 */

/* Stand-in for `#include <rcl/types.h>` having already been processed. */
#define RCL_RET_OK 0

#include "nros/rcl_compat.h"

int nros_rcl_compat_collision_probe(void);
int nros_rcl_compat_collision_probe(void) {
    return RCL_RET_OK;
}
