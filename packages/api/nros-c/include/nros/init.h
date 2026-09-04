/**
 * @file init.h
 * @ingroup grp_init
 * @brief Library initialisation and support context.
 *
 * The support context (`nros_support_t`) is the entry point for all
 * nros operations.  It manages the middleware session (zenoh-pico) and
 * must be initialised before any nodes, publishers, or subscriptions
 * are created.
 *
 * Typical usage:
 * @code
 * nros_support_t support = nros_support_get_zero_initialized();
 * nros_support_init(&support, NULL, 0);
 * // ... create nodes, publishers, etc.
 * nros_support_fini(&support);
 * @endcode
 */

#ifndef NROS_INIT_H
#define NROS_INIT_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/init.h>` continues to compile. */
#include "nros/types.h"

#endif /* NROS_INIT_H */
