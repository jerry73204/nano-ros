/// @file main.c
/// @brief C service client example - calls AddTwoInts service (blocking)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

// nros modular includes (rclc-style)
#include <nros/app_main.h>
#include <nros/check.h>
#include <nros/client.h>
#include <nros/executor.h>
#include <nros/init.h>
#include <nros/node.h>

// Generated C bindings for example_interfaces/srv/AddTwoInts
#include "example_interfaces.h"

// ----------------------------------------------------------------------------
// Application state
// ----------------------------------------------------------------------------

static struct {
    nros_support_t support;
    nros_node_t node;
    nros_client_t client;
    nros_executor_t executor;
} app;

// ----------------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------------

int nros_app_main(int argc, char** argv) {
    // Line-buffer stdout: glibc full-buffers non-tty stdout, so when piped to
    // a test harness each line must flush on its newline.
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

    printf("nros C Service Client (AddTwoInts)\n");
    printf("=====================================\n");

    // Operands from the first two positional args (default: 2 3).
    int64_t a = 2;
    int64_t b = 3;
    if (argc >= 3) {
        char* end_a = NULL;
        char* end_b = NULL;
        long long parsed_a = strtoll(argv[1], &end_a, 10);
        long long parsed_b = strtoll(argv[2], &end_b, 10);
        if (end_a && *end_a == '\0' && end_b && *end_b == '\0') {
            a = (int64_t)parsed_a;
            b = (int64_t)parsed_b;
        }
    }

    // Get configuration from environment
    const char* locator = getenv("NROS_LOCATOR");
    if (!locator) {
        locator = NROS_ENTRY_LOCATOR;
    }

    const char* domain_str = getenv("ROS_DOMAIN_ID");
    uint8_t domain_id = (uint8_t)NROS_ENTRY_DOMAIN_ID;
    if (domain_str) {
        domain_id = (uint8_t)atoi(domain_str);
    }

    printf("Locator: %s\n", locator);
    printf("Domain ID: %d\n", domain_id);

    // Zero-initialize all static state
    memset(&app, 0, sizeof(app));

    // Build type info using generated type name/hash
    nros_service_type_t add_two_ints_type = {
        .type_name = example_interfaces_srv_add_two_ints_get_type_name(),
        .type_hash = example_interfaces_srv_add_two_ints_get_type_hash(),
    };

    NROS_CHECK_RET(nros_support_init(&app.support, locator, domain_id), 1);
    printf("Support initialized\n");
    NROS_CHECK_RET(rclc_node_init_default(&app.node, "add_two_ints_client", "/", &app.support), 1);
    printf("Node created: %s\n", rcl_node_get_name(&app.node));

    NROS_CHECK_RET(
        rclc_client_init_default(&app.client, &app.node, &add_two_ints_type, "/add_two_ints"), 1);
    printf("Client created for service: %s\n", rcl_client_get_service_name(&app.client));

    // Clients must be registered with an executor before use.
    NROS_CHECK_RET(nros_executor_init(&app.executor, &app.support, 4), 1);
    NROS_CHECK_RET(nros_executor_add_client(&app.executor, &app.client), 1);

    int exit_code = 0;

    // Prepare the single request using the generated type
    example_interfaces_srv_add_two_ints_request request;
    example_interfaces_srv_add_two_ints_request_init(&request);
    request.a = a;
    request.b = b;

    // Serialize request using generated function
    uint8_t req_buf[256];
    size_t req_len = 0;
    int32_t req_len_rc = example_interfaces_srv_add_two_ints_request_serialize(
        &request, req_buf, sizeof(req_buf), &req_len);
    if (req_len_rc != 0) {
        fprintf(stderr, "Failed to serialize request\n");
        exit_code = 1;
    } else {
        // Call service (blocking)
        uint8_t resp_buf[256];
        size_t resp_len = 0;
        nros_ret_t ret =
            nros_client_call(&app.client, req_buf, req_len, resp_buf, sizeof(resp_buf), &resp_len);

        if (ret == NROS_RET_OK) {
            // Deserialize response using generated function
            example_interfaces_srv_add_two_ints_response response;
            if (example_interfaces_srv_add_two_ints_response_deserialize(&response, resp_buf,
                                                                         resp_len) == 0) {
                printf("Result of add_two_ints: %lld\n", (long long)response.sum);
            } else {
                fprintf(stderr, "Failed to deserialize response\n");
                exit_code = 1;
            }
        } else if (ret == NROS_RET_TIMEOUT) {
            fprintf(stderr, "Service call timed out (is the server running?)\n");
            exit_code = 1;
        } else {
            fprintf(stderr, "Service call failed with error %d\n", ret);
            exit_code = 1;
        }
    }

    // Cleanup
    printf("\nShutting down...\n");
    rclc_executor_fini(&app.executor);
    nros_client_fini(&app.client);
    rcl_node_fini(&app.node);
    rclc_support_fini(&app.support);

    printf("Goodbye!\n");
    return exit_code;
}

NROS_APP_MAIN_REGISTER()
