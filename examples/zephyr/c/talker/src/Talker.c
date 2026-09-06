/// @file Talker.c
/// @brief Zephyr C talker — typed component (RFC-0043 / phase-244.C2).
///
/// `talker_configure` creates a publisher on `/chatter` + a timer that
/// publishes the official ROS 2 demo payload (`std_msgs/String`,
/// `Hello World: N`) each tick, via the GENERATED typed C bindings
/// (`nros_find_interfaces(LANGUAGE C)` in CMakeLists — phase-277 W4; was
/// hand-rolled CDR). `NROS_C_COMPONENT` emits the C-ABI factory/configure the
/// Zephyr typed Entry carrier (the shared entry pack) calls.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <nros/component.h>

#include <std_msgs/msg/std_msgs_msg_string.h>

typedef struct {
    _Alignas(8) uint8_t pub[NROS_C_PUBLISHER_STORAGE_SIZE];
    int32_t count;
    /* Message + CDR scratch live in the component (not the tick stack):
     * String is ~256 B and RTOS timer/app stacks are small. */
    std_msgs_msg_string msg;
    uint8_t buf[300];
} talker_t;

static void on_tick(void* ctx) {
    talker_t* self = (talker_t*)ctx;
    /* Pre-increment so the first payload is "Hello World: 1", matching the
     * official ROS 2 demo talker. */
    self->count++;
    std_msgs_msg_string_init(&self->msg);
    snprintf(self->msg.data, sizeof(self->msg.data), "Hello World: %d", (int)self->count);
    /* Serialize via the generated typed serializer, publish through the
     * component (nros-cpp) raw seam. Buffer: 4-byte CDR header + 4-byte
     * length + payload + NUL. */
    size_t len = 0;
    if (std_msgs_msg_string_serialize(&self->msg, self->buf, sizeof(self->buf), &len) == 0 &&
        nros_cpp_publish_raw(self->pub, self->buf, len) == 0) {
        printf("Publishing: '%s'\n", self->msg.data);
    }
}

static nros_ret_t talker_configure(const nros_cpp_node_t* node, void* executor, talker_t* self) {
    setvbuf(stdout, NULL, _IONBF, 0);
    self->count = 0;
    int32_t rc = nros_cpp_publisher_create(node, "/chatter", std_msgs_msg_string_get_type_name(),
                                           "", nros_c_qos_default(), self->pub);
    if (rc != 0) {
        return rc;
    }
    size_t timer_handle;
    return nros_cpp_timer_create(executor, /*period_ms=*/500, on_tick, self, &timer_handle);
}

NROS_C_COMPONENT(talker_t, talker_configure)
