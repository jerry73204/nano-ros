/// @file main.c
/// @brief C parameters example — exercises the nros C parameter server.
///
/// Declares bool / integer / double / string parameters, reads them back,
/// updates values, and prints the results (plus a small clock-API demo).
/// Extracted from the pre-phase-277 `examples/native/c/talker` demo block so
/// the talker stays a minimal chatter publisher (parity with
/// `examples/native/cpp/parameters`). The example exits with status 0 only
/// when every roundtrip passes — non-zero exit codes encode which assertion
/// failed. Used by the `c_parameters` integration test.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nros/clock.h>
#include <nros/parameter.h>

static int run(void) {
    // Clock demo: read the system clock once.
    nros_clock_t clock;
    if (nros_clock_init(&clock, NROS_CLOCK_SYSTEM_TIME) == NROS_RET_OK) {
        nros_time_t now;
        if (nros_clock_get_now(&clock, &now) == NROS_RET_OK) {
            printf("System time: %d.%09u sec\n", now.sec, now.nanosec);
        }
        (void)rcl_clock_fini(&clock);
    }

    // Parameter server backed by static storage (no heap). The server struct
    // must start zero-initialized — `init` rejects one that looks live.
    static nros_parameter_t storage[8];
    nros_parameter_server_t params = nros_parameter_server_get_zero_initialized();
    if (nros_parameter_server_init(&params, storage, 8) != NROS_RET_OK) {
        fprintf(stderr, "param server init failed\n");
        return 1;
    }

    // Declare parameters with default values.
    if (nros_parameter_declare_bool(&params, "verbose", false) != NROS_RET_OK) return 1;
    if (nros_parameter_declare_integer(&params, "publish_rate_hz", 1) != NROS_RET_OK) return 1;
    if (nros_parameter_declare_double(&params, "scale_factor", 1.0) != NROS_RET_OK) return 1;
    if (nros_parameter_declare_string(&params, "topic_name", "/chatter") != NROS_RET_OK) return 1;

    // Read back and display parameter values.
    bool verbose = true;
    int64_t rate_hz = 0;
    double scale = 0.0;
    char topic[64] = {0};

    if (nros_parameter_get_bool(&params, "verbose", &verbose) != NROS_RET_OK) return 2;
    if (nros_parameter_get_integer(&params, "publish_rate_hz", &rate_hz) != NROS_RET_OK) return 2;
    if (nros_parameter_get_double(&params, "scale_factor", &scale) != NROS_RET_OK) return 2;
    if (nros_parameter_get_string(&params, "topic_name", topic, sizeof(topic)) != NROS_RET_OK) {
        return 2;
    }

    printf("Parameters: verbose=%s, rate=%lld Hz, scale=%.2f, topic=%s\n",
           verbose ? "true" : "false", (long long)rate_hz, scale, topic);

    if (verbose != false) return 3;
    if (rate_hz != 1) return 3;
    if (scale < 0.99 || scale > 1.01) return 3;
    if (strcmp(topic, "/chatter") != 0) return 3;

    // Update values and read them back.
    if (nros_parameter_set_bool(&params, "verbose", true) != NROS_RET_OK) return 4;
    if (nros_parameter_get_bool(&params, "verbose", &verbose) != NROS_RET_OK) return 4;
    if (verbose != true) return 4;
    printf("After set: verbose=%s\n", verbose ? "true" : "false");

    if (nros_parameter_set_integer(&params, "publish_rate_hz", 10) != NROS_RET_OK) return 4;
    if (nros_parameter_get_integer(&params, "publish_rate_hz", &rate_hz) != NROS_RET_OK) return 4;
    if (rate_hz != 10) return 4;

    if (nros_parameter_set_string(&params, "topic_name", "/rosout") != NROS_RET_OK) return 4;
    if (nros_parameter_get_string(&params, "topic_name", topic, sizeof(topic)) != NROS_RET_OK) {
        return 4;
    }
    if (strcmp(topic, "/rosout") != 0) return 4;

    // Unknown parameters must be rejected, not invented.
    if (nros_parameter_get_bool(&params, "missing", &verbose) == NROS_RET_OK) return 5;

    (void)nros_parameter_server_fini(&params);

    printf("OK verbose=%s rate=%lld topic=%s\n", verbose ? "true" : "false", (long long)rate_hz,
           topic);
    return 0;
}

int main(void) {
    // Line-buffer stdout: glibc full-buffers non-tty stdout, so when piped to
    // a test harness each line must flush on its newline.
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif
    return run();
}
