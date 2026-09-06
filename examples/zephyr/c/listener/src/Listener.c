/// @file Listener.c
/// @brief Zephyr C listener — typed component (RFC-0043 / phase-244.C2).
///
/// A stateful C component: `listener_configure` binds the `on_raw` callback (by
/// identity, fn-ptr + self ctx) as a raw zero-copy subscription on `/chatter`,
/// decoding each sample with the GENERATED typed C deserializer
/// (`nros_find_interfaces(LANGUAGE C)` in CMakeLists — phase-277 W4; was
/// hand-rolled CDR) and logging the official ROS 2 demo line
/// (`I heard: [Hello World: N]`). `NROS_C_COMPONENT` emits the C-ABI factory +
/// configure the Zephyr typed Entry carrier (the shared entry pack)
/// calls. No declarative descriptor, no synthesizing interpreter, no callback
/// name.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <nros/component.h>

#include <std_msgs/msg/std_msgs_msg_string.h>

typedef struct {
    int recv;
    /* Decoded message lives in the component (not the callback stack):
     * String is ~256 B and RTOS dispatch stacks are small. */
    std_msgs_msg_string msg;
} listener_t;

static void on_raw(const uint8_t* data, size_t len, void* ctx) {
    listener_t* self = (listener_t*)ctx;
    std_msgs_msg_string_init(&self->msg);
    if (std_msgs_msg_string_deserialize(&self->msg, data, len) == 0) {
        printf("I heard: [%s]\n", self->msg.data);
        self->recv++;
    }
}

static nros_ret_t listener_configure(const nros_cpp_node_t* node, void* executor,
                                     listener_t* self) {
    setvbuf(stdout, NULL, _IONBF, 0);
    (void)executor; /* node-scoped sub; executor unused */
    size_t handle;
    int32_t rc =
        nros_cpp_subscription_register(node, "/chatter", std_msgs_msg_string_get_type_name(), "",
                                       nros_c_qos_default(), on_raw, self, &handle,
                                       /*options=*/NULL); /* phase-402: NULL = defaults */
    if (rc == 0) {
        /* Readiness markers the harness greps before driving the talker.
         * The first is the SHARED one every nano-ros listener prints
         * (`nros_tests::output::LISTENER_READY_MARKER`, phase-342 W7); the
         * second is zephyr.rs's own `NODE_READY_MARKER` and stays until that
         * file moves onto the shared table. */
        printf("Subscriber created for topic: %s\n", "/chatter");
        printf("Waiting for messages\n");
    }
    return rc;
}

NROS_C_COMPONENT(listener_t, listener_configure)
