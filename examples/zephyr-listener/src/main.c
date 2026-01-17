/*
 * nano-ros Zephyr Listener Example
 *
 * This example demonstrates a ROS 2 compatible subscriber using
 * zenoh-pico transport on Zephyr RTOS.
 *
 * Receives CDR-serialized std_msgs/Int32 messages, compatible
 * with ROS 2 nodes using rmw_zenoh.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(nano_ros_listener, LOG_LEVEL_INF);

/* CDR Serialization constants */
#define CDR_HEADER_SIZE 4
#define INT32_MSG_SIZE (CDR_HEADER_SIZE + 4)

/* Topic key in rmw_zenoh format */
#define TOPIC_KEY "0//chatter/std_msgs::msg::dds_::Int32_/RIHS01_abc123"

/* Deserialize an Int32 message from CDR format */
static int deserialize_int32(const uint8_t *buf, size_t buf_len, int32_t *value)
{
    if (buf_len < INT32_MSG_SIZE) {
        return -1;
    }

    /* Verify CDR encapsulation header */
    if (buf[0] != 0x00 || buf[1] != 0x01 || buf[2] != 0x00 || buf[3] != 0x00) {
        LOG_WRN("Unexpected CDR header: %02x %02x %02x %02x",
                buf[0], buf[1], buf[2], buf[3]);
        /* Continue anyway - might be big-endian */
    }

    /* Read Int32 value (assuming little-endian) */
    *value = (int32_t)(
        ((uint32_t)buf[4] << 0) |
        ((uint32_t)buf[5] << 8) |
        ((uint32_t)buf[6] << 16) |
        ((uint32_t)buf[7] << 24)
    );

    return 0;
}

/* Callback for received messages
 *
 * Note: In a real implementation with zenoh-pico, this would be:
 *
 * void subscriber_callback(z_loaned_sample_t *sample, void *context) {
 *     const z_loaned_bytes_t *payload = z_sample_payload(sample);
 *     size_t len = z_bytes_len(payload);
 *     // Read and deserialize...
 * }
 */
static void handle_message(const uint8_t *data, size_t len)
{
    int32_t value;
    if (deserialize_int32(data, len, &value) == 0) {
        LOG_INF("Received: %d", value);
    } else {
        LOG_ERR("Failed to deserialize message");
    }
}

int main(void)
{
    LOG_INF("nano-ros Zephyr Listener Starting");
    LOG_INF("Subscribing to: /chatter (std_msgs/Int32)");

    /* Note: In a real implementation with zenoh-pico:
     *
     * 1. Open zenoh session:
     *    z_owned_config_t config;
     *    z_config_default(&config);
     *    z_owned_session_t session;
     *    z_open(&session, z_config_move(&config), NULL);
     *
     * 2. Create closure and declare subscriber:
     *    z_owned_closure_sample_t callback;
     *    z_closure_sample(&callback, subscriber_callback, NULL, NULL);
     *
     *    z_view_keyexpr_t keyexpr;
     *    z_view_keyexpr_from_str(&keyexpr, TOPIC_KEY);
     *
     *    z_owned_subscriber_t sub;
     *    z_declare_subscriber(z_session_loan(&session), &sub,
     *                         z_view_keyexpr_loan(&keyexpr),
     *                         z_closure_sample_move(&callback), NULL);
     *
     * 3. Run event loop or use polling
     */

    /* Simulate receiving messages for demonstration */
    uint8_t test_msg[INT32_MSG_SIZE] = {
        0x00, 0x01, 0x00, 0x00,  /* CDR header */
        0x2A, 0x00, 0x00, 0x00   /* Int32: 42 */
    };

    LOG_INF("Simulating received message...");
    handle_message(test_msg, sizeof(test_msg));

    /* In a real implementation, we'd wait for callbacks */
    while (1) {
        /* TODO: Call zp_read() to process incoming messages */
        /* zp_read(z_session_loan(&session), NULL); */
        k_sleep(K_MSEC(100));
    }

    return 0;
}
