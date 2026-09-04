/*
 * The parameter family's entry points, pinned by SIGNATURE.
 *
 * Phase 379 W5 renamed the family `nros_param_*` -> `nros_parameter_*` (ledger
 * row `c:parameter_server_t`) and kept the old spellings one release as
 * `NROS_DEPRECATED_MSG` `static inline` forwarders. Phase-417 stage 6 step B
 * RETIRED those forwarders and the four deprecated TYPE aliases beside them,
 * so this TU no longer has a second half to pin: what survives is clause 1,
 * every live entry point with the signature the header documents.
 *
 * That is still worth a gate. Compile-only (no main): taking a function
 * POINTER forces a real lookup and a real signature match, so a declaration
 * whose argument list drifts from the Rust definition cbindgen emitted fails
 * HERE rather than at some consumer's call site.
 */

#include "nros/parameter.h"

/* The live entry points, with the documented signatures. */
static struct nros_parameter_server_t (*const k_new_server_get_zero_initialized)(void) =
    nros_parameter_server_get_zero_initialized;
static nros_ret_t (*const k_new_server_init)(struct nros_parameter_server_t*,
                                             struct nros_parameter_t*,
                                             size_t) = nros_parameter_server_init;
static nros_ret_t (*const k_new_server_set_callback)(struct nros_parameter_server_t*,
                                                     nros_parameter_callback_t,
                                                     void*) = nros_parameter_server_set_callback;
static nros_ret_t (*const k_new_declare_bool)(struct nros_parameter_server_t*, const char*,
                                              bool) = nros_parameter_declare_bool;
static nros_ret_t (*const k_new_declare_integer)(struct nros_parameter_server_t*, const char*,
                                                 int64_t) = nros_parameter_declare_integer;
static nros_ret_t (*const k_new_declare_double)(struct nros_parameter_server_t*, const char*,
                                                double) = nros_parameter_declare_double;
static nros_ret_t (*const k_new_declare_string)(struct nros_parameter_server_t*, const char*,
                                                const char*) = nros_parameter_declare_string;
static nros_ret_t (*const k_new_get_bool)(const struct nros_parameter_server_t*, const char*,
                                          bool*) = nros_parameter_get_bool;
static nros_ret_t (*const k_new_get_integer)(const struct nros_parameter_server_t*, const char*,
                                             int64_t*) = nros_parameter_get_integer;
static nros_ret_t (*const k_new_get_double)(const struct nros_parameter_server_t*, const char*,
                                            double*) = nros_parameter_get_double;
static nros_ret_t (*const k_new_get_string)(const struct nros_parameter_server_t*, const char*,
                                            char*, size_t) = nros_parameter_get_string;
static nros_ret_t (*const k_new_set_bool)(struct nros_parameter_server_t*, const char*,
                                          bool) = nros_parameter_set_bool;
static nros_ret_t (*const k_new_set_integer)(struct nros_parameter_server_t*, const char*,
                                             int64_t) = nros_parameter_set_integer;
static nros_ret_t (*const k_new_set_double)(struct nros_parameter_server_t*, const char*,
                                            double) = nros_parameter_set_double;
static nros_ret_t (*const k_new_set_string)(struct nros_parameter_server_t*, const char*,
                                            const char*) = nros_parameter_set_string;
static nros_ret_t (*const k_new_declare_byte_array)(struct nros_parameter_server_t*, const char*,
                                                    const uint8_t*,
                                                    size_t) = nros_parameter_declare_byte_array;
static nros_ret_t (*const k_new_declare_bool_array)(struct nros_parameter_server_t*, const char*,
                                                    const bool*,
                                                    size_t) = nros_parameter_declare_bool_array;
static nros_ret_t (*const k_new_declare_integer_array)(struct nros_parameter_server_t*, const char*,
                                                       const int64_t*, size_t) =
    nros_parameter_declare_integer_array;
static nros_ret_t (*const k_new_declare_double_array)(struct nros_parameter_server_t*, const char*,
                                                      const double*,
                                                      size_t) = nros_parameter_declare_double_array;
static nros_ret_t (*const k_new_declare_string_array)(struct nros_parameter_server_t*, const char*,
                                                      const char* const*,
                                                      size_t) = nros_parameter_declare_string_array;
static nros_ret_t (*const k_new_get_byte_array)(const struct nros_parameter_server_t*, const char*,
                                                const uint8_t**,
                                                size_t*) = nros_parameter_get_byte_array;
static nros_ret_t (*const k_new_get_bool_array)(const struct nros_parameter_server_t*, const char*,
                                                const bool**,
                                                size_t*) = nros_parameter_get_bool_array;
static nros_ret_t (*const k_new_get_integer_array)(const struct nros_parameter_server_t*,
                                                   const char*, const int64_t**,
                                                   size_t*) = nros_parameter_get_integer_array;
static nros_ret_t (*const k_new_get_double_array)(const struct nros_parameter_server_t*,
                                                  const char*, const double**,
                                                  size_t*) = nros_parameter_get_double_array;
static nros_ret_t (*const k_new_get_string_array)(const struct nros_parameter_server_t*,
                                                  const char*, const char* const**,
                                                  size_t*) = nros_parameter_get_string_array;
static nros_ret_t (*const k_new_set_byte_array)(struct nros_parameter_server_t*, const char*,
                                                const uint8_t*,
                                                size_t) = nros_parameter_set_byte_array;
static nros_ret_t (*const k_new_set_bool_array)(struct nros_parameter_server_t*, const char*,
                                                const bool*,
                                                size_t) = nros_parameter_set_bool_array;
static nros_ret_t (*const k_new_set_integer_array)(struct nros_parameter_server_t*, const char*,
                                                   const int64_t*,
                                                   size_t) = nros_parameter_set_integer_array;
static nros_ret_t (*const k_new_set_double_array)(struct nros_parameter_server_t*, const char*,
                                                  const double*,
                                                  size_t) = nros_parameter_set_double_array;
static nros_ret_t (*const k_new_set_string_array)(struct nros_parameter_server_t*, const char*,
                                                  const char* const*,
                                                  size_t) = nros_parameter_set_string_array;
static bool (*const k_new_has)(const struct nros_parameter_server_t*,
                               const char*) = nros_parameter_has;
static enum nros_parameter_type_t (*const k_new_get_type)(const struct nros_parameter_server_t*,
                                                          const char*) = nros_parameter_get_type;
static size_t (*const k_new_server_get_count)(const struct nros_parameter_server_t*) =
    nros_parameter_server_get_count;
static nros_ret_t (*const k_new_server_fini)(struct nros_parameter_server_t*) =
    nros_parameter_server_fini;

/* Silence "defined but not used" without needing a main(). */
const void* nros_param_entry_point_probe(void);
const void* nros_param_entry_point_probe(void) {
    (void)k_new_server_get_zero_initialized;
    (void)k_new_server_init;
    (void)k_new_server_set_callback;
    (void)k_new_declare_bool;
    (void)k_new_declare_integer;
    (void)k_new_declare_double;
    (void)k_new_declare_string;
    (void)k_new_get_bool;
    (void)k_new_get_integer;
    (void)k_new_get_double;
    (void)k_new_get_string;
    (void)k_new_set_bool;
    (void)k_new_set_integer;
    (void)k_new_set_double;
    (void)k_new_set_string;
    (void)k_new_declare_byte_array;
    (void)k_new_declare_bool_array;
    (void)k_new_declare_integer_array;
    (void)k_new_declare_double_array;
    (void)k_new_declare_string_array;
    (void)k_new_get_byte_array;
    (void)k_new_get_bool_array;
    (void)k_new_get_integer_array;
    (void)k_new_get_double_array;
    (void)k_new_get_string_array;
    (void)k_new_set_byte_array;
    (void)k_new_set_bool_array;
    (void)k_new_set_integer_array;
    (void)k_new_set_double_array;
    (void)k_new_set_string_array;
    (void)k_new_has;
    (void)k_new_get_type;
    (void)k_new_server_get_count;
    (void)k_new_server_fini;
    return (const void*)k_new_server_fini;
}
