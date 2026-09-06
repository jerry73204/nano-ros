/// @file Talker.c
/// @brief nano-ros Zephyr BYO starter — zenoh C talker, typed component
/// (RFC-0043 / phase-244.C4).
///
/// `talker_configure` creates a raw publisher on `/chatter` + a timer that
/// publishes a CDR-encoded Int32 counter each tick. Copy this app, edit the
/// topic / message / logic. `NROS_C_COMPONENT` emits the C-ABI factory/configure
/// the Zephyr typed Entry carrier (the shared entry pack) calls;
/// the carrier runs it on the real executor (ZephyrBoard::run_components) — no
/// hand-written `nros_app_main`. Booting needs a zenoh router at
/// CONFIG_NROS_ZENOH_LOCATOR (e.g. `ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7456`)."];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <std_msgs/std_msgs.h>
#include <nros/component.h>

typedef struct {
    _Alignas(8) uint8_t pub[NROS_C_PUBLISHER_STORAGE_SIZE];
    int32_t count;
} talker_t;

static void on_tick(void* ctx) {
    talker_t* self = (talker_t*)ctx;
    std_msgs_msg_int32 msg;
    std_msgs_msg_int32_init(&msg);
    msg.data = self->count;
    uint8_t buf[16];
    size_t n = 0;
    if (std_msgs_msg_int32_serialize(&msg, buf, sizeof(buf), &n) == 0 &&
        nros_cpp_publish_raw(self->pub, buf, n) == 0) {
        printf("Published: %d\n", (int)self->count);
    }
    self->count++;
}

static nros_ret_t talker_configure(const nros_cpp_node_t* node, void* executor, talker_t* self) {
    setvbuf(stdout, NULL, _IONBF, 0);
    self->count = 0;
    int32_t rc = nros_cpp_publisher_create(node, "/chatter", std_msgs_msg_int32_get_type_name(), "",
                                           nros_c_qos_default(), self->pub);
    if (rc != 0) {
        return rc;
    }
    size_t timer_handle;
    return nros_cpp_timer_create(executor, /*period_ms=*/1000, on_tick, self, &timer_handle);
}

NROS_C_COMPONENT(talker_t, talker_configure)
