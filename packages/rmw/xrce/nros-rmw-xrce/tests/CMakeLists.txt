# Phase 115.K.2 scaffold smoke test.
# Confirms the static library links and the register entry point
# returns NROS_RMW_RET_OK against a stub `nros_rmw_cffi_register`
# defined in the test TU itself (the real symbol lives in the
# nros-rmw-cffi Rust crate; the scaffold doesn't link against the
# Rust runtime).

# phase-420 W9 step 4 — `nros_rmw_xrce` is now an INTERFACE target over the
# CARGO LANE'S archive, so what this binary links is what an image ships. It
# also carries every include directory a consumer of that archive needs (the
# nros-rmw-cffi + platform headers, the vendored public and private headers, and
# the GENERATED `<uxr/client/config.h>` those objects were compiled against), so
# the restatements that used to live here are gone.
add_executable(nros_rmw_xrce_c_smoke smoke.c)
target_link_libraries(nros_rmw_xrce_c_smoke PRIVATE nros_rmw_xrce)

add_test(NAME nros_rmw_xrce_c_smoke COMMAND nros_rmw_xrce_c_smoke)

# Issue 0847 — an entity handle outliving its session must not fault.
#
# Needs `internal.h` (the session/entity states are private to the backend), and
# no agent: a CLOSED session's destructor must not touch the transport, which is
# the invariant under test. Before the fix this binary SEGFAULTED rather than
# reporting, so ctest's exit-status verdict is what makes it a test at all --
# the issue asks for exactly that, because a crash after the last line of output
# reads as a clean run to anything that greps stdout.
add_executable(nros_rmw_xrce_entity_lifetime entity_lifetime.c)
# `internal.h` includes <uxr/client/client.h> and the private backend headers.
# Those used to be PRIVATE on a STATIC `nros_rmw_xrce` and were restated here;
# since W9 step 4 the target is an INTERFACE over the cargo archive and carries
# them, INCLUDING the generated config headers that archive was built against —
# so this TU and those objects agree on `uxr_session_t`'s layout by
# construction rather than by two `configure_file`s happening to match.
target_link_libraries(nros_rmw_xrce_entity_lifetime PRIVATE nros_rmw_xrce)

add_test(NAME nros_rmw_xrce_entity_lifetime COMMAND nros_rmw_xrce_entity_lifetime)
