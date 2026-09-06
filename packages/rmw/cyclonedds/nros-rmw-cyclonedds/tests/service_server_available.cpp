// phase-428 W13.c — `service_server_is_available` answers from the DDS
// discovery cache, in both directions.
//
// Three assertions, in order, on ONE client:
//   1. no server anywhere       → OK, false   (not UNSUPPORTED: the slot is filled)
//   2. a server comes up        → OK, true    (within a discovery window)
//   3. that server is destroyed → OK, false   (CURRENT, not latched — the thing
//                                              issue 1087 was about)
//
// The server lives on a SECOND session (= its own participant), because that
// is the case `wait_for_service` exists for: a server in another process. A
// same-participant pair would match through Cyclone's local delivery and never
// exercise SEDP. Cross-participant discovery is where `service_concurrent`
// flakes, so the window here is polled (up to 10 s, 20 ms steps), not slept,
// and the assertion is on the FIRST true — a slow match costs time, not a red.

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include "nros/rmw_ret.h"
#include "nros/rmw_vtable.h"
#include "nros_rmw_cyclonedds.h"
#include "nros_test_domain.h"

namespace {
const nros_rmw_vtable_t* g_vt = nullptr;

constexpr int kPollStepMs = 20;
constexpr int kPollBudgetMs = 10000;

// Poll `service_server_is_available` until it reports `want`, or the budget
// runs out. Every call must return OK — an error is a failed test, not a
// retry. Returns true when `want` was observed.
bool poll_until(const rmw_client_t* cli, bool want, const char* what) {
    for (int elapsed = 0; elapsed <= kPollBudgetMs; elapsed += kPollStepMs) {
        bool avail = !want;
        rmw_ret_t rc = g_vt->service_server_is_available(cli, &avail);
        if (rc != NROS_RMW_RET_OK) {
            std::fprintf(stderr, "%s: service_server_is_available returned %d\n", what, (int)rc);
            return false;
        }
        if (avail == want) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollStepMs));
    }
    std::fprintf(stderr, "%s: still %s after %d ms\n", what, want ? "unavailable" : "available",
                 kPollBudgetMs);
    return false;
}
} // namespace

extern "C" rmw_ret_t nros_rmw_cffi_register_named(const char* /*name*/,
                                                  const nros_rmw_vtable_t* vt) {
    g_vt = vt;
    return NROS_RMW_RET_OK;
}

int main() {
    if (nros_rmw_cyclonedds_register() != NROS_RMW_RET_OK || g_vt == nullptr) {
        std::fprintf(stderr, "register failed\n");
        return 1;
    }
    if (g_vt->service_server_is_available == nullptr) {
        std::fprintf(stderr, "service_server_is_available slot is NULL\n");
        return 2;
    }

    const uint32_t domain = nros_test_domain(99);

    // --- client side: its own session/participant ---------------------------
    rmw_session_t cs{};
    cs.node_name = "server_available_client";
    cs.namespace_ = "/";
    if (g_vt->create_session(nullptr, 0, domain, cs.node_name, nullptr, &cs) != NROS_RMW_RET_OK) {
        std::fprintf(stderr, "client create_session failed\n");
        return 3;
    }
    rmw_node_t cnode{};
    cnode.name = cs.node_name;
    cnode.namespace_ = cs.namespace_;
    cnode.session = &cs;

    rmw_client_t cli{};
    cli.service_name = "server_available";
    cli.type_name = "nros_test::srv::dds_::AddTwoInts";
    const rmw_service_type_support_t cts{cli.type_name, ""};
    if (g_vt->create_client(&cnode, &cts, cli.service_name, domain, nullptr, &cli) !=
        NROS_RMW_RET_OK) {
        std::fprintf(stderr, "create_client failed\n");
        (void)g_vt->destroy_session(&cs);
        return 4;
    }

    // (1) Nobody serves this name: false, and OK — not UNSUPPORTED, not true.
    {
        bool avail = true;
        rmw_ret_t rc = g_vt->service_server_is_available(&cli, &avail);
        if (rc != NROS_RMW_RET_OK) {
            std::fprintf(stderr, "no-server: returned %d, want OK\n", (int)rc);
            return 5;
        }
        if (avail) {
            std::fprintf(stderr, "no-server: reported available with no server on the bus\n");
            return 6;
        }
    }
    // NULL out-pointer is INVALID_ARGUMENT, not a write through NULL.
    if (g_vt->service_server_is_available(&cli, nullptr) != NROS_RMW_RET_INVALID_ARGUMENT) {
        std::fprintf(stderr, "NULL out_available should be INVALID_ARGUMENT\n");
        return 7;
    }

    // --- server side: a SECOND session/participant --------------------------
    rmw_session_t ss{};
    ss.node_name = "server_available_server";
    ss.namespace_ = "/";
    if (g_vt->create_session(nullptr, 0, domain, ss.node_name, nullptr, &ss) != NROS_RMW_RET_OK) {
        std::fprintf(stderr, "server create_session failed\n");
        return 8;
    }
    rmw_node_t snode{};
    snode.name = ss.node_name;
    snode.namespace_ = ss.namespace_;
    snode.session = &ss;

    rmw_service_t srv{};
    srv.service_name = "server_available";
    srv.type_name = "nros_test::srv::dds_::AddTwoInts";
    const rmw_service_type_support_t sts{srv.type_name, ""};
    if (g_vt->create_service(&snode, &sts, srv.service_name, domain, nullptr, &srv) !=
        NROS_RMW_RET_OK) {
        std::fprintf(stderr, "create_service failed\n");
        return 9;
    }

    // (2) The server's request reader matches our writer AND its reply writer
    // matches our reader — both halves, across SEDP.
    if (!poll_until(&cli, true, "server-up")) return 10;

    // A server of a DIFFERENT name on the same bus is not this one: still true
    // for ours, and this is also the "unrelated endpoints do not count" half —
    // an `other` client with no server of ITS name must read false while the
    // bus is busy.
    rmw_client_t other{};
    other.service_name = "server_available_nobody";
    other.type_name = "nros_test::srv::dds_::AddTwoInts";
    const rmw_service_type_support_t ots{other.type_name, ""};
    if (g_vt->create_client(&cnode, &ots, other.service_name, domain, nullptr, &other) !=
        NROS_RMW_RET_OK) {
        std::fprintf(stderr, "create_client(other) failed\n");
        return 11;
    }
    {
        bool avail = true;
        if (g_vt->service_server_is_available(&other, &avail) != NROS_RMW_RET_OK || avail) {
            std::fprintf(stderr, "other-name: a server of another name counted as ours\n");
            return 12;
        }
    }
    g_vt->destroy_client(&other);

    // (3) The server goes away: unavailable again. Nothing is latched.
    g_vt->destroy_service(&srv);
    if (!poll_until(&cli, false, "server-down")) return 13;

    (void)g_vt->destroy_session(&ss);
    g_vt->destroy_client(&cli);
    (void)g_vt->destroy_session(&cs);
    std::printf("OK\n");
    return 0;
}
