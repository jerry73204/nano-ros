/**
 * @file parameter.h
 * @ingroup grp_parameter
 * @brief Parameter server API.
 *
 * Declare, get, and set typed parameters on a local parameter server.
 */

#ifndef NROS_PARAMETER_H
#define NROS_PARAMETER_H

#include "nros/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Phase 91.C1: type definitions (nros_parameter_server_state_t,
 * nros_parameter_type_t, nros_parameter_array_t, nros_parameter_value_t,
 * nros_parameter_t, nros_parameter_callback_t, nros_parameter_server_t) come
 * from <nros/nros_generated.h> via the nros/types.h include above.
 *
 * The typed parameter setters / getters (bool / integer / double /
 * *_array) are declared by hand below; the auto-generated header
 * cannot synthesise them from the runtime side, so this file keeps
 * the canonical declarations.
 */

/* ===================================================================
 * Functions
 * =================================================================== */

/**
 * @brief Get a zero-initialized parameter server.
 * @return Zero-initialized `nros_parameter_server_t`.
 */
NROS_PUBLIC struct nros_parameter_server_t nros_parameter_server_get_zero_initialized(void);

/**
 * @brief Initialise a parameter server with user-provided storage.
 *
 * @param server   Pointer to a zero-initialized parameter server.
 * @param storage  Pointer to a user-provided parameter array.
 * @param capacity Maximum number of parameters the array can hold.
 *
 * @retval NROS_RET_OK on success.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_server_init(struct nros_parameter_server_t* server,
                                      struct nros_parameter_t* storage, size_t capacity);

/**
 * @brief Set a parameter change callback.
 *
 * @param server   Pointer to an initialized parameter server.
 * @param callback Callback function, or NULL to clear.
 * @param context  User context.
 *
 * @retval NROS_RET_OK on success.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_server_set_callback(struct nros_parameter_server_t* server,
                                              nros_parameter_callback_t callback, void* context);

/**
 * @brief Declare a boolean parameter.
 *
 * @param server        Pointer to an initialized parameter server.
 * @param name          Parameter name (null-terminated).
 * @param default_value Default boolean value.
 *
 * @retval NROS_RET_OK on success.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_declare_bool(struct nros_parameter_server_t* server, const char* name,
                                       bool default_value);

/**
 * @brief Declare an integer parameter.
 *
 * @param server        Pointer to an initialized parameter server.
 * @param name          Parameter name (null-terminated).
 * @param default_value Default integer value.
 *
 * @retval NROS_RET_OK on success.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_declare_integer(struct nros_parameter_server_t* server, const char* name,
                                          int64_t default_value);

/**
 * @brief Declare a double parameter.
 *
 * @param server        Pointer to an initialized parameter server.
 * @param name          Parameter name (null-terminated).
 * @param default_value Default double value.
 *
 * @retval NROS_RET_OK on success.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_declare_double(struct nros_parameter_server_t* server, const char* name,
                                         double default_value);

/**
 * @brief Declare a string parameter.
 *
 * @param server        Pointer to an initialized parameter server.
 * @param name          Parameter name (null-terminated).
 * @param default_value Default string value (null-terminated).
 *
 * @retval NROS_RET_OK on success.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_declare_string(struct nros_parameter_server_t* server, const char* name,
                                         const char* default_value);

/**
 * @brief Get a boolean parameter value.
 *
 * @param server Pointer to an initialized parameter server.
 * @param name   Parameter name.
 * @param value  Output: parameter value.
 *
 * @retval NROS_RET_OK        on success.
 * @retval NROS_RET_NOT_FOUND  if the parameter does not exist.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_get_bool(const struct nros_parameter_server_t* server, const char* name,
                                   bool* value);

/**
 * @brief Get an integer parameter value.
 *
 * @param server Pointer to an initialized parameter server.
 * @param name   Parameter name.
 * @param value  Output: parameter value.
 *
 * @retval NROS_RET_OK        on success.
 * @retval NROS_RET_NOT_FOUND  if the parameter does not exist.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_get_integer(const struct nros_parameter_server_t* server,
                                      const char* name, int64_t* value);

/**
 * @brief Get a double parameter value.
 *
 * @param server Pointer to an initialized parameter server.
 * @param name   Parameter name.
 * @param value  Output: parameter value.
 *
 * @retval NROS_RET_OK        on success.
 * @retval NROS_RET_NOT_FOUND  if the parameter does not exist.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_get_double(const struct nros_parameter_server_t* server, const char* name,
                                     double* value);

/**
 * @brief Get a string parameter value.
 *
 * @param server  Pointer to an initialized parameter server.
 * @param name    Parameter name.
 * @param value   Output buffer for the string.
 * @param max_len Maximum length of the output buffer.
 *
 * @retval NROS_RET_OK        on success.
 * @retval NROS_RET_NOT_FOUND  if the parameter does not exist.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_get_string(const struct nros_parameter_server_t* server, const char* name,
                                     char* value, size_t max_len);

/**
 * @brief Set a boolean parameter value.
 *
 * @param server Pointer to an initialized parameter server.
 * @param name   Parameter name.
 * @param value  New boolean value.
 *
 * @retval NROS_RET_OK        on success.
 * @retval NROS_RET_NOT_FOUND  if the parameter does not exist.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_set_bool(struct nros_parameter_server_t* server, const char* name,
                                   bool value);

/**
 * @brief Set an integer parameter value.
 *
 * @param server Pointer to an initialized parameter server.
 * @param name   Parameter name.
 * @param value  New integer value.
 *
 * @retval NROS_RET_OK        on success.
 * @retval NROS_RET_NOT_FOUND  if the parameter does not exist.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_set_integer(struct nros_parameter_server_t* server, const char* name,
                                      int64_t value);

/**
 * @brief Set a double parameter value.
 *
 * @param server Pointer to an initialized parameter server.
 * @param name   Parameter name.
 * @param value  New double value.
 *
 * @retval NROS_RET_OK        on success.
 * @retval NROS_RET_NOT_FOUND  if the parameter does not exist.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_set_double(struct nros_parameter_server_t* server, const char* name,
                                     double value);

/**
 * @brief Set a string parameter value.
 *
 * @param server Pointer to an initialized parameter server.
 * @param name   Parameter name.
 * @param value  New string value (null-terminated).
 *
 * @retval NROS_RET_OK        on success.
 * @retval NROS_RET_NOT_FOUND  if the parameter does not exist.
 */
NROS_PUBLIC
nros_ret_t nros_parameter_set_string(struct nros_parameter_server_t* server, const char* name,
                                     const char* value);

/* -------------------------------------------------------------------
 * Array parameters
 *
 * Array parameters store a pointer + length to caller-owned data.
 * The caller MUST keep the underlying storage alive for the lifetime of
 * the parameter (until @ref nros_parameter_server_fini, or until the
 * parameter is overwritten with a new pointer via the matching `_set`
 * function). String arrays point to an array of `const char*` — each
 * element is itself a null-terminated, caller-owned string.
 * ------------------------------------------------------------------- */

/** @brief Declare a byte array parameter (`uint8_t[]`). */
NROS_PUBLIC
nros_ret_t nros_parameter_declare_byte_array(struct nros_parameter_server_t* server,
                                             const char* name, const uint8_t* data, size_t len);
/** @brief Declare a boolean array parameter (`bool[]`). */
NROS_PUBLIC
nros_ret_t nros_parameter_declare_bool_array(struct nros_parameter_server_t* server,
                                             const char* name, const bool* data, size_t len);
/** @brief Declare an integer array parameter (`int64_t[]`). */
NROS_PUBLIC
nros_ret_t nros_parameter_declare_integer_array(struct nros_parameter_server_t* server,
                                                const char* name, const int64_t* data, size_t len);
/** @brief Declare a double array parameter (`double[]`). */
NROS_PUBLIC
nros_ret_t nros_parameter_declare_double_array(struct nros_parameter_server_t* server,
                                               const char* name, const double* data, size_t len);
/** @brief Declare a string array parameter (array of `const char*`). */
NROS_PUBLIC
nros_ret_t nros_parameter_declare_string_array(struct nros_parameter_server_t* server,
                                               const char* name, const char* const* data,
                                               size_t len);

/**
 * @name Array parameter pointer identity (issue 0340)
 *
 * The array `declare` / `set` calls above take BORROWED storage: the server
 * records the caller's `data` pointer verbatim and never copies the elements,
 * which is why the caller must keep that memory valid for the parameter's
 * lifetime. The `get` calls below return **that same pointer**, unchanged —
 * not a copy, not an interior pointer, not server-owned storage.
 *
 * That identity is LOAD-BEARING, not incidental. `nros/parameter.hpp`'s
 * `ParameterServer` allocates array blocks out of a header-side pool with an
 * out-of-band capacity word written immediately in front of each block, and
 * recovers the capacity on `set` by reading back from the returned pointer.
 * If this side ever starts copying arrays, returning an interior pointer, or
 * handing back storage it owns, that read lands on unrelated memory.
 *
 * The C++ side now verifies the returned pointer lies inside its own pool
 * before trusting it, so a violation surfaces as `NROS_RET_INVALID_ARGUMENT`
 * rather than an out-of-bounds read — but the guarantee is still part of this
 * ABI. Changing it requires changing `parameter.hpp` in the same commit.
 *
 * The proper fix is to carry the capacity in server-owned state (a field on
 * `nros_parameter_array_t`, or a `nros_parameter_get_*_array_capacity()` accessor).
 * That is a breaking change to a hand-mirrored FFI struct and a public C ABI,
 * so it is deferred rather than done silently.
 * @{
 */

/** @brief Get a byte array parameter (returns stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_get_byte_array(const struct nros_parameter_server_t* server,
                                         const char* name, const uint8_t** data, size_t* len);
/** @brief Get a boolean array parameter (returns stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_get_bool_array(const struct nros_parameter_server_t* server,
                                         const char* name, const bool** data, size_t* len);
/** @brief Get an integer array parameter (returns stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_get_integer_array(const struct nros_parameter_server_t* server,
                                            const char* name, const int64_t** data, size_t* len);
/** @brief Get a double array parameter (returns stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_get_double_array(const struct nros_parameter_server_t* server,
                                           const char* name, const double** data, size_t* len);
/** @brief Get a string array parameter (returns stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_get_string_array(const struct nros_parameter_server_t* server,
                                           const char* name, const char* const** data, size_t* len);
/** @} */

/** @brief Set a byte array parameter (replaces stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_set_byte_array(struct nros_parameter_server_t* server, const char* name,
                                         const uint8_t* data, size_t len);
/** @brief Set a boolean array parameter (replaces stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_set_bool_array(struct nros_parameter_server_t* server, const char* name,
                                         const bool* data, size_t len);
/** @brief Set an integer array parameter (replaces stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_set_integer_array(struct nros_parameter_server_t* server,
                                            const char* name, const int64_t* data, size_t len);
/** @brief Set a double array parameter (replaces stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_set_double_array(struct nros_parameter_server_t* server, const char* name,
                                           const double* data, size_t len);
/** @brief Set a string array parameter (replaces stored pointer + length). */
NROS_PUBLIC
nros_ret_t nros_parameter_set_string_array(struct nros_parameter_server_t* server, const char* name,
                                           const char* const* data, size_t len);

/**
 * @brief Check if a parameter exists.
 *
 * @param server Pointer to an initialized parameter server.
 * @param name   Parameter name.
 * @return @c true if the parameter exists, @c false otherwise.
 */
NROS_PUBLIC bool nros_parameter_has(const struct nros_parameter_server_t* server, const char* name);

/**
 * @brief Get the type of a parameter.
 *
 * @param server Pointer to an initialized parameter server.
 * @param name   Parameter name.
 * @return Parameter type, or `NROS_PARAMETER_NOT_SET` if not found.
 */
NROS_PUBLIC
enum nros_parameter_type_t nros_parameter_get_type(const struct nros_parameter_server_t* server,
                                                   const char* name);

/**
 * @brief Get the number of declared parameters.
 *
 * @param server Pointer to an initialized parameter server.
 * @return Number of parameters.
 */
NROS_PUBLIC size_t nros_parameter_server_get_count(const struct nros_parameter_server_t* server);

/**
 * @brief Finalise a parameter server.
 *
 * @param server Pointer to an initialized parameter server.
 * @retval NROS_RET_OK on success.
 */
NROS_PUBLIC nros_ret_t nros_parameter_server_fini(struct nros_parameter_server_t* server);

/* ===================================================================
 * Service-Backed Parameter API (requires NROS_PARAM_SERVICES feature)
 *
 * These functions operate on the nros-params::ParameterServer owned by
 * the Executor. After calling nros_executor_register_parameter_services,
 * declared parameters are visible to `ros2 param list /<node>`.
 *
 * Only available when nros-c is built with the `param-services` Cargo
 * feature (requires alloc).
 * =================================================================== */

struct nros_executor_t;

/**
 * @brief Register the 6 ROS 2 parameter services on the executor's node.
 *
 * Creates service servers for:
 *   - `~/get_parameters`
 *   - `~/set_parameters`
 *   - `~/set_parameters_atomically`
 *   - `~/list_parameters`
 *   - `~/describe_parameters`
 *   - `~/get_parameter_types`
 *
 * After this call, parameters declared via
 * nros_executor_declare_param_*() are visible to `ros2 param` tooling.
 */
NROS_PUBLIC nros_ret_t nros_executor_register_parameter_services(struct nros_executor_t* executor);

/** @brief Declare a boolean parameter on the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_declare_param_bool(struct nros_executor_t* executor,
                                                        const char* name, bool value);
/** @brief Declare an integer parameter on the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_declare_param_integer(struct nros_executor_t* executor,
                                                           const char* name, int64_t value);
/** @brief Declare a double parameter on the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_declare_param_double(struct nros_executor_t* executor,
                                                          const char* name, double value);
/** @brief Declare a string parameter on the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_declare_param_string(struct nros_executor_t* executor,
                                                          const char* name, const char* value);

/** @brief Get a boolean parameter from the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_get_param_bool(struct nros_executor_t* executor,
                                                    const char* name, bool* out_value);
/** @brief Get an integer parameter from the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_get_param_integer(struct nros_executor_t* executor,
                                                       const char* name, int64_t* out_value);
/** @brief Get a double parameter from the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_get_param_double(struct nros_executor_t* executor,
                                                      const char* name, double* out_value);
/** @brief Get a string parameter into a caller-provided null-terminated buffer. */
NROS_PUBLIC nros_ret_t nros_executor_get_param_string(struct nros_executor_t* executor,
                                                      const char* name, char* out_value,
                                                      size_t max_len);

/** @brief Set a boolean parameter on the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_set_param_bool(struct nros_executor_t* executor,
                                                    const char* name, bool value);
/** @brief Set an integer parameter on the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_set_param_integer(struct nros_executor_t* executor,
                                                       const char* name, int64_t value);
/** @brief Set a double parameter on the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_set_param_double(struct nros_executor_t* executor,
                                                      const char* name, double value);
/** @brief Set a string parameter on the executor's server. */
NROS_PUBLIC nros_ret_t nros_executor_set_param_string(struct nros_executor_t* executor,
                                                      const char* name, const char* value);

/** @brief Check if a parameter exists on the executor's server. */
NROS_PUBLIC bool nros_executor_has_param(struct nros_executor_t* executor, const char* name);

#ifdef __cplusplus
}
#endif

#endif /* NROS_PARAMETER_H */
