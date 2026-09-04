/**
 * @file subscription.h
 * @ingroup grp_pubsub
 * @brief Topic subscription API.
 *
 * Create subscriptions with nros_subscription_init() and receive
 * deserialised messages via a user-provided callback.
 *
 * For manual polling, create the subscription with
 * nros_subscription_init_polling() and drain it with
 * nros_subscription_take_serialized() — or
 * nros_subscription_take_sequence() for a batch and
 * nros_subscription_take_validated() for the E2E-safety variant.
 */

#ifndef NROS_SUBSCRIPTION_H
#define NROS_SUBSCRIPTION_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/subscription.h>` continues to compile. */
#include "nros/types.h"

#endif /* NROS_SUBSCRIPTION_H */
