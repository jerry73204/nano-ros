/**
 * @file service.h
 * @ingroup grp_service
 * @brief Service server API.
 *
 * Create service servers with nros_service_init().  For executor-driven
 * dispatch — the usual shape — register a `nros_service_callback_t` at init
 * time and let the executor deliver requests and send responses for you.
 *
 * For manual polling, create the server with nros_service_init_polling(),
 * take requests with nros_service_take_request_raw(), and send responses
 * with nros_service_send_response_raw().
 *
 * For a TYPED handler — a deserialised request in, a typed response out,
 * with no hand-written CDR — the generated per-service header emits
 * `<Srv>_service_handler_t`, `<Srv>_service_handler_init()` and
 * `<Srv>_service_init()`; see `packs/c/service.h.jinja` (phase-417 W5.e).
 * Those are `static inline` glue over the same entry points documented
 * here, so the two paths cannot diverge.
 *
 * (The `c:take_request` question in the phase-379 parity ledger is
 * ANSWERED: `nros_service_take_request()` is now a deprecated forwarder
 * onto `nros_service_take_request_raw()` — see below.)
 */

#ifndef NROS_SERVICE_H
#define NROS_SERVICE_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/service.h>` continues to compile. */
#include "nros/types.h"

#endif /* NROS_SERVICE_H */
