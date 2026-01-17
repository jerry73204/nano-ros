/*
 * nano-ros Zephyr Talker Example
 *
 * This example demonstrates a ROS 2 compatible publisher using
 * zenoh-pico transport on Zephyr RTOS.
 *
 * The message format is CDR-serialized std_msgs/Int32, compatible
 * with ROS 2 nodes using rmw_zenoh.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nano_ros_talker, LOG_LEVEL_INF);

/* Note: When zenoh-pico is available as a Zephyr module, include:
 * #include <zenoh-pico.h>
 *
 * For now, this is a placeholder that demonstrates the structure.
 */

/* CDR Serialization for std_msgs/Int32
 *
 * CDR Format:
 *   - 4 bytes: encapsulation header (0x00, 0x01, 0x00, 0x00 for LE)
 *   - 4 bytes: int32 data (little-endian)
 */
#define CDR_HEADER_SIZE 4
#define INT32_MSG_SIZE (CDR_HEADER_SIZE + 4)

/* Topic key in rmw_zenoh format:
 * <domain_id>/<topic_name>/<type_name>/RIHS01_<type_hash>
 */
#define TOPIC_KEY "0//chatter/std_msgs::msg::dds_::Int32_/RIHS01_abc123"

/* Serialize an Int32 message to CDR format */
static int serialize_int32(uint8_t *buf, size_t buf_len, int32_t value)
{
    if (buf_len < INT32_MSG_SIZE) {
        return -1;
    }

    /* CDR encapsulation header (little-endian) */
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x00;
    buf[3] = 0x00;

    /* Int32 value (little-endian) */
    buf[4] = (value >> 0) & 0xFF;
    buf[5] = (value >> 8) & 0xFF;
    buf[6] = (value >> 16) & 0xFF;
    buf[7] = (value >> 24) & 0xFF;

    return INT32_MSG_SIZE;
}

int main(void)
{
    LOG_INF("nano-ros Zephyr Talker Starting");
    LOG_INF("Topic: /chatter (std_msgs/Int32)");

    int32_t counter = 0;
    uint8_t msg_buf[INT32_MSG_SIZE];

    /* Note: In a real implementation with zenoh-pico:
     *
     * 1. Open zenoh session:
     *    z_owned_config_t config;
     *    z_config_default(&config);
     *    z_owned_session_t session;
     *    z_open(&session, z_config_move(&config), NULL);
     *
     * 2. Declare publisher:
     *    z_view_keyexpr_t keyexpr;
     *    z_view_keyexpr_from_str(&keyexpr, TOPIC_KEY);
     *    z_owned_publisher_t pub;
     *    z_declare_publisher(z_session_loan(&session), &pub,
     *                        z_view_keyexpr_loan(&keyexpr), NULL);
     *
     * 3. Publish messages in loop:
     *    z_owned_bytes_t payload;
     *    z_bytes_copy_from_buf(&payload, msg_buf, msg_len);
     *    z_publisher_put(z_publisher_loan(&pub), z_bytes_move(&payload), NULL);
     */

    while (1) {
        /* Serialize message */
        int msg_len = serialize_int32(msg_buf, sizeof(msg_buf), counter);
        if (msg_len < 0) {
            LOG_ERR("Serialization failed");
            continue;
        }

        LOG_INF("Publishing: %d (CDR bytes: %02x %02x %02x %02x %02x %02x %02x %02x)",
                counter,
                msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3],
                msg_buf[4], msg_buf[5], msg_buf[6], msg_buf[7]);

        /* TODO: Actually publish via zenoh-pico */
        /* z_publisher_put(...) */

        counter++;
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
