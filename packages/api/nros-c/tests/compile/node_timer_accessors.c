/* phase-417 W5.c — the node and timer accessors that were ledgered `gap` must
 * be DECLARED, callable, and spelled the way a C user will type them.
 *
 * Why this probe exists, concretely: every function here lives in a Rust
 * source file and reaches C only through cbindgen. The `graph_query_entry_points`
 * probe next door records what that failure looks like — a family that compiled,
 * passed `just check c`, and emitted NO header declarations, so it was
 * uncallable from C while the Rust side stayed green the whole time. Taking
 * function POINTERS here turns a missing or drifted declaration into a build
 * failure instead of a discovery by a user.
 *
 * What this TU can and cannot prove. It is compiled `-fsyntax-only`, like its
 * siblings — a probe that LINKED would need a registered RMW backend
 * (`nros_app_register_backends`), which is a fixture, not a syntax check. So
 * the assertions here are about SHAPE, and the assertions about VALUES live in
 * `packages/api/nros-c/src/timer.rs`'s and `node.rs`'s unit tests, which run in
 * the same lane and need no backend. The two halves are complementary: the
 * unit tests cannot see a missing cbindgen declaration, and this TU cannot see
 * a wrong answer.
 */

#include <nros/nros.h>
#include <nros/timer.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int main(void) {
    /* --- node accessors (ledger: c:node_is_valid, c:node_get_domain_id,
     * c:node_get_fully_qualified_name, c:node_resolve_name) --------------- */

    /* `bool`, not `nros_ret_t`: rcl's `rcl_node_is_valid` is a predicate, and
     * a predicate over a handle we hold cannot fail to answer. phase-417
     * stage 6 gave it rcl's NAME too — the old `nros_node_is_valid` is a
     * deprecated forwarder in `<nros/node.h>`. */
    bool (*p_is_valid)(const struct nros_node_t*) = rcl_node_is_valid;

    /* Out-param + status, NOT a bare `uint32_t` return. 0 is a legal ROS
     * domain, so a bare return would have to spend it on "cannot answer" —
     * which is the defect issue 1008 was. If someone "simplifies" this to a
     * value return, this line stops compiling. */
    /* NOT renamed to `rcl_node_get_domain_id`: upstream writes a `size_t *`
     * and ours writes a `uint32_t *`. In C that mismatch is a WARNING, so a
     * ported `size_t id; rcl_node_get_domain_id(&node, &id);` would compile
     * and leave half of `id` uninitialised — the out-parameter type is
     * authored at the CALL SITE, which is what separates this from the
     * typesupport handle RFC-0089 settled we keep. */
    nros_ret_t (*p_domain)(const struct nros_node_t*, uint32_t*) = nros_node_get_domain_id;

    /* Caller-owned buffer, not rcl's `const char *` return: we have no
     * allocator and the node struct holds no composed FQN. */
    nros_ret_t (*p_fqn)(const struct nros_node_t*, char*, size_t) =
        nros_node_get_fully_qualified_name;

    /* `only_expand` is rcl's own parameter and keeps rcl's meaning. rcl's
     * `allocator` and `is_service` are deliberately absent — the first
     * because there is no allocator, the second because it selects between
     * validators we do not ship, and an argument we ignore is the inert-
     * parameter defect RFC-0089 is written to end. */
    nros_ret_t (*p_resolve)(const struct nros_node_t*, const char*, bool, char*, size_t) =
        nros_node_resolve_name;

    /* --- timer accessors (ledger: c:timer_is_canceled, c:timer_is_ready,
     * c:timer_get_time_since_last_call) ----------------------------------- */

    nros_ret_t (*p_canceled)(const struct nros_timer_t*, bool*) = rcl_timer_is_canceled;
    nros_ret_t (*p_ready)(const struct nros_timer_t*, bool*) = rcl_timer_is_ready;
    nros_ret_t (*p_since)(const struct nros_timer_t*, uint64_t*) =
        nros_timer_get_time_since_last_call;

    /* --- nros_difference_times: Time - Time -> Duration ------------------ */

    /* A `static inline` in <nros/timer.h>, so there is no exported symbol and
     * cbindgen has nothing to emit — but its SHAPE can still drift, and this
     * is the shape that matters. rcl's is
     * `(const rcl_time_point_t *, const rcl_time_point_t *, rcl_duration_t *)`:
     * two POINTERS to time points and a duration out-param. The neighbouring
     * `nros_time_sub` takes `(nros_time_t, nros_duration_t)` BY VALUE and
     * computes a different operation (Time - Duration -> Time), which is what
     * ledger row `c:difference_times` had to be corrected for once already.
     * Taking the address here is what makes a drift toward that shape a build
     * failure. */
    nros_ret_t (*p_difference)(const struct nros_time_t*, const struct nros_time_t*,
                               struct nros_duration_t*) = nros_difference_times;

    /* And it must be CALLABLE with that shape — the pointer above would still
     * bind if an argument type were merely compatible. */
    struct nros_time_t start = nros_time_from_nanoseconds((int64_t)1000000000);
    struct nros_time_t finish = nros_time_from_nanoseconds((int64_t)2500000000);
    struct nros_duration_t delta;
    nros_ret_t rc = nros_difference_times(&start, &finish, &delta);

    (void)p_is_valid;
    (void)p_domain;
    (void)p_fqn;
    (void)p_resolve;
    (void)p_canceled;
    (void)p_ready;
    (void)p_since;
    (void)p_difference;
    (void)delta;

    return rc == NROS_RET_OK ? 0 : 1;
}
