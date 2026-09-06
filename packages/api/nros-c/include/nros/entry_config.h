/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file entry_config.h
 * @brief The compile-time connect configuration an entry bakes: `NROS_ENTRY_LOCATOR`
 *        and `NROS_ENTRY_DOMAIN_ID`.
 *
 * ONE derivation, in a C header, because both entry languages need it.
 *
 * ## Why this file exists
 *
 * The ladder used to live in `<nros/main.hpp>` — a C++ header — while the C
 * sibling `<nros/app_main.h>` defined the same two macros as `""` and `0` with
 * no derivation at all. Both are consumed as VALUES by `run_components` /
 * `run_tiers` and by the CLI entry codegen, so the two spellings never
 * conflicted at link time; they simply disagreed about what the image dials.
 *
 * Today that costs nothing, because an embedded C entry is routed to the C++
 * emitter and compiles as a `.cpp` that includes `main.hpp`. The moment a pure
 * C entry becomes possible (phase-432 W3.1) it stops being free: the image
 * would compile, link, boot — and dial nothing, because `""` reaches the
 * backend as "absent". That is a byte-for-byte re-run of issue #174, which the
 * XRCE arm below exists to fix, and it would arrive with no diagnostic.
 *
 * Same class as issue 0946 one layer down: that gate made the CMAKE-side
 * locator ladders one producer. This makes the HEADER-side ones one producer.
 *
 * ## The contract
 *
 * Preprocessor only — no types, no includes, no linkage. Both `main.hpp` and
 * `app_main.h` include it at the point their own ladder used to sit, so an
 * entry may still bake a literal by defining either macro BEFORE the include
 * and the `#ifndef` guards leave it alone.
 *
 * Both macros must keep expanding to a value of the right kind:
 * `NROS_ENTRY_LOCATOR` to a STRING LITERAL (never a nullable pointer — callers
 * pass it by value) and `NROS_ENTRY_DOMAIN_ID` to an integer constant
 * expression.
 */
#ifndef NROS_ENTRY_CONFIG_H
#define NROS_ENTRY_CONFIG_H

/* ---- NROS_ENTRY_LOCATOR ----
 *
 * Phase 244.C2 — the compile-time connect locator for Zephyr (and NuttX, which
 * reuses this macro). Defaults to the Kconfig `CONFIG_NROS_ZENOH_LOCATOR` when
 * the board sets one (the e2e gate threads
 * `CONFIG_NROS_ZENOH_LOCATOR=tcp/127.0.0.1:<port>` per fixture), else `""`.
 *
 * Issue 0330 — the bottom rung is the EMPTY STRING, not a router endpoint.
 * This header is RMW-blind and must not restate a backend fact; `""` is the
 * established "absent — let the backend discover" value. It travels through
 * `nros_support_init` / `nros_cpp_init` to the RFC-0045 resolver's empty bottom
 * rung and then to the linked backend, which substitutes its own default
 * (zenoh: `nros_rmw_zenoh::DEFAULT_LOCATOR`).
 */
#ifndef NROS_ENTRY_LOCATOR
#if defined(CONFIG_NROS_ZENOH_LOCATOR)
#define NROS_ENTRY_LOCATOR CONFIG_NROS_ZENOH_LOCATOR
#elif defined(CONFIG_NROS_RMW_XRCE) && defined(CONFIG_NROS_XRCE_AGENT_ADDR) &&                     \
    defined(CONFIG_NROS_XRCE_AGENT_PORT)
/* #174 / phase-286 W3 — XRCE has NO zenoh locator; its agent endpoint lives in
 * CONFIG_NROS_XRCE_AGENT_{ADDR,PORT}. Without this, `NROS_ENTRY_LOCATOR` fell
 * to `""` and the C/C++ XRCE entry opened its session with no agent address —
 * the transport never connected (`run_components` rc=-100 TRANSPORT_ERROR, 0
 * delivery). Synthesize the bare `host:port` the XRCE session parser accepts
 * (`nros-rmw-xrce/session.c` `parse_host_port`) — the C/C++ analog of the Rust
 * example `build.rs` bake (issue #163, which fixed only the Rust images).
 * Adjacent string-literal concat + stringize: "127.0.0.1" ":" 2018 →
 * "127.0.0.1:2018". */
#define NROS_ENTRY_LOCATOR_STRINGIZE_(x) #x
#define NROS_ENTRY_LOCATOR_STRINGIZE(x) NROS_ENTRY_LOCATOR_STRINGIZE_(x)
#define NROS_ENTRY_LOCATOR                                                                         \
    CONFIG_NROS_XRCE_AGENT_ADDR ":" NROS_ENTRY_LOCATOR_STRINGIZE(CONFIG_NROS_XRCE_AGENT_PORT)
#else
#define NROS_ENTRY_LOCATOR ""
#endif
#endif /* NROS_ENTRY_LOCATOR */

/* ---- NROS_ENTRY_DOMAIN_ID ----
 *
 * Compile-time on embedded, never a runtime env (the CLAUDE.md embedded
 * domain-id rule). Cyclone keys off `CONFIG_NROS_CYCLONE_DOMAIN_ID` when
 * present (matching ASI), else the generic `CONFIG_NROS_DOMAIN_ID`.
 *
 * Never pin the Cyclone knob to a literal in a board conf: the phase-180
 * split-brain silently ran every cyclone image on domain 0.
 */
#ifndef NROS_ENTRY_DOMAIN_ID
#if defined(NROS_RMW_CYCLONEDDS) && defined(CONFIG_NROS_CYCLONE_DOMAIN_ID)
#define NROS_ENTRY_DOMAIN_ID CONFIG_NROS_CYCLONE_DOMAIN_ID
#elif defined(CONFIG_NROS_DOMAIN_ID)
#define NROS_ENTRY_DOMAIN_ID CONFIG_NROS_DOMAIN_ID
#else
#define NROS_ENTRY_DOMAIN_ID 0
#endif
#endif /* NROS_ENTRY_DOMAIN_ID */

#endif /* NROS_ENTRY_CONFIG_H */
