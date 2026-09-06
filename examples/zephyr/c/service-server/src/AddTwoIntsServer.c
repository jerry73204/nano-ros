/// @file AddTwoIntsServer.c
/// @brief Zephyr C AddTwoInts service server — typed component.
///
/// `server_configure` binds `handle_add` (by identity) as a raw callback-style
/// service on `/add_two_ints`; the real handler decodes the CDR request
/// (int64 a, b) and writes the CDR reply (int64 sum). `NROS_C_COMPONENT` emits
/// the C-ABI factory + configure seam the Zephyr typed Entry carrier
/// (the shared entry pack) drives. No interpreter synthesis.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <nros/component.h>

typedef struct {
    int served;
} add_server_t;

static int64_t read_i64_le(const uint8_t* p) {
    uint64_t v = 0;
    int i;
    for (i = 0; i < 8; ++i) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return (int64_t)v;
}

static void write_i64_le(uint8_t* p, int64_t x) {
    uint64_t v = (uint64_t)x;
    int i;
    for (i = 0; i < 8; ++i) {
        p[i] = (uint8_t)(v >> (8 * i));
    }
}

static bool handle_add(const uint8_t* req, size_t req_len, uint8_t* resp, size_t resp_cap,
                       size_t* resp_len, void* ctx) {
    add_server_t* self = (add_server_t*)ctx;
    /* Request CDR: 4-byte encap header, then int64 a (off 4), int64 b (off 12). */
    if (req_len < 20 || resp_cap < 12) {
        return false;
    }
    int64_t a = read_i64_le(req + 4);
    int64_t b = read_i64_le(req + 12);
    int64_t sum = a + b;
    resp[0] = req[0];
    resp[1] = req[1];
    resp[2] = req[2];
    resp[3] = req[3];
    write_i64_le(resp + 4, sum);
    *resp_len = 12;
    self->served++;
    printf("Incoming request\na: %lld b: %lld\n", (long long)a, (long long)b);
    return true;
}

static nros_ret_t server_configure(const nros_cpp_node_t* node, void* executor,
                                   add_server_t* self) {
    setvbuf(stdout, NULL, _IONBF, 0);
    (void)executor; /* node-scoped service; executor unused */
    size_t handle;
    int32_t rc =
        nros_cpp_service_server_register(node, "/add_two_ints", "example_interfaces/srv/AddTwoInts",
                                         "", nros_c_qos_default(), handle_add, self,
                                         /*sched_context=*/0, &handle);
    if (rc == 0) {
        /* Readiness marker the e2e harness greps before driving the client. */
        printf("Waiting for service requests\n");
    }
    return rc;
}

NROS_C_COMPONENT(add_server_t, server_configure)
