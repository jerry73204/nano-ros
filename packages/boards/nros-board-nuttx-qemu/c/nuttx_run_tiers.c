/*
 * nuttx_run_tiers.c — phase-281 W3 (nuttx): C++ embedded multi-tier entry for NuttX.
 *
 * Implements `nros_board_nuttx_run_tiers`, the NuttX analog of the FreeRTOS
 * `nros_board_freertos_run_tiers` and the Zephyr `nros_board_zephyr_run_tiers`:
 * opens ONE RMW session on the calling thread (the NuttX `app_main` thread — an
 * already-running post-boot task), runs the boot tier's setup, then CHAIN-spawns
 * the remaining tiers (each with a borrowed executor sharing the one session) so
 * their setups serialize (issue #144), and runs the boot tier on the caller.
 *
 * NuttX deltas vs the FreeRTOS / Zephyr mirrors:
 *   - NuttX is POSIX, so non-boot tiers spawn via `pthread_create` (NOT
 *     xTaskCreate / k_thread and NOT a tier shim): each tier gets its own
 *     `pthread_attr_t` with `pthread_attr_setstacksize` (tier.stack_bytes, else
 *     a 16 KiB default — the executor working set is heap-allocated via
 *     `nros_platform_alloc`, so the tier stack only carries call frames) and
 *     `SCHED_FIFO` + `pthread_attr_setschedparam` at the tier's RAW NuttX
 *     priority (`[tiers.*.nuttx].priority` verbatim), with
 *     `PTHREAD_EXPLICIT_SCHED` so the attr's policy/priority actually take;
 *   - the boot tier adopts its declared RAW priority on the caller thread via
 *     `pthread_setschedparam(pthread_self(), SCHED_FIFO, …)` (the spawned tiers
 *     get theirs at `pthread_create`);
 *   - NuttX owns boot + networking: the board FFI `main` brings up `eth0`
 *     (virtio-net) BEFORE `app_main` runs (phase-280), so there is no network
 *     bring-up here — just the weak `nros_board_network_wait()` gate (mirrors
 *     `NuttxBoard::run_components` in main.hpp), not the FreeRTOS lwIP path.
 *
 * Called from `NuttxBoard::run_tiers` (main.hpp) via the generated
 * `nros_app_main` (RFC-0015 Model 1 embedded, RFC-0043 typed path — the NuttX
 * startup path calls `app_main`, like FreeRTOS, NOT Zephyr's `main(void)`).
 *
 * phase-281 W3 / RFC-0015 §5.
 */

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* --- nros-cpp C FFI forward declarations ---
 *
 * These symbols are defined in nros-cpp (Rust) and linked into the final image.
 * No need to include nros_cpp_ffi.h here — the linker resolves by name.
 * Signatures must match nros_cpp_ffi.h exactly. */

extern int nros_cpp_init(const char* locator, uint8_t domain_id, const char* node_name,
                         const char* namespace_, void* storage);

extern int nros_cpp_fini(void* storage);

extern void* nros_cpp_executor_session_handle(void* executor);

extern int nros_cpp_executor_open_over_session(void* session_handle, const char* node_name,
                                               uint32_t domain_id, void* out_storage);

extern int nros_cpp_executor_set_active_groups(void* executor, const char* const* groups, size_t n);

extern int nros_cpp_spin_once(void* handle, int32_t timeout_ms);

/* issue 0636 option 3 — the tier spin loop's scheduled gap.
 *
 * ONE implementation, shared with the Rust tier runners
 * (`nros_platform::board::tier`): a second spelling per language is how the
 * tier-priority marker drifted, which is most of what this issue was. Pass 0
 * on the first call and keep the returned value; the sleep, when the rule
 * calls for one, happens inside. State is an opaque u64 rather than a struct
 * so there is no hand-mirrored FFI layout to drift (three prior incidents). */
extern uint64_t nros_tier_spin_gap_step(uint64_t state, uint64_t iter_start_ns, uint64_t now_ns,
                                        uint32_t spin_period_us);
extern uint64_t nros_platform_clock_ns(void);


/* nros_board_network_wait: weak no-op in <nros/main.h> (phase-432 W3.1 moved
 * it there from the C++ sibling main.hpp so a pure C entry links); strong
 * override on boards that must block for link-up. On the canonical NuttX path
 * the board FFI `main` already brought up eth0 before app_main (phase-280), so
 * this is a no-op — the kernel-side network config runs before the app entry. */
extern void nros_board_network_wait(void);

/* RFC-0034 — the sole sanctioned allocation seam (wraps the NuttX heap); a
 * single counter sees all traffic. Direct malloc/free are forbidden. */
extern void* nros_platform_alloc(size_t size);
extern void nros_platform_dealloc(void* ptr);

/* --- Executor storage sizing ---
 *
 * nros_cpp_init / nros_cpp_executor_open_over_session both need storage of
 * CPP_EXECUTOR_OPAQUE_U64S * 8 bytes, 8-byte aligned. The exact per-build value
 * is in the cmake-generated nros_cpp_config_generated.h; to keep this seam
 * standalone-compilable (mirroring freertos_run_tiers.c / zephyr_run_tiers.c,
 * which cannot include the generated header) we use the NuttX/embedded ARM
 * fallback (79304 bytes, nros_cpp_config_generated_nuttx.h) rounded up to 80 KiB
 * for headroom. nros_platform_alloc on the NuttX heap returns aligned memory. */
/* issue #245 — prefer the REAL per-build executor size when the generated
 * header is visible to this compile; the hardcoded fallback silently went
 * 32 bytes short on Zephyr when the executor grew (heap-corruption crash).
 * If this platform's executor outgrows the fallback, the same corruption
 * follows — keep the generated-header path working. */
#if defined(__has_include)
#if __has_include(<nros/nros_cpp_config_generated.h>)
#include <nros/nros_cpp_config_generated.h>
#endif
#endif
#ifdef NROS_CPP_EXECUTOR_STORAGE_SIZE
#define NROS_NUTTX_EXECUTOR_STORAGE_BYTES ((NROS_CPP_EXECUTOR_STORAGE_SIZE + 7u) & ~7u)
#else
#define NROS_NUTTX_EXECUTOR_STORAGE_BYTES 81920u
#endif

/* --- Per-tier pthread stack ---
 *
 * The borrowed executor's working set (the 80 KiB above + zenoh-pico buffers)
 * lives on the heap via nros_platform_alloc, so a tier pthread's stack only
 * carries call frames — 16 KiB is a sane default. `[tiers.*.nuttx].stack_bytes`
 * overrides it when set. NOTE (mirrors the freertos seam's stack note): this
 * default is untuned for the full run_tiers path under QEMU; runtime-proving
 * (the next wave) may need to raise it. */
#define NROS_NUTTX_TIER_STACK_BYTES 16384u

/* --- Local tier-spec type ---
 *
 * Mirror of nros_native_tier_spec_t (nros/main.h). Layout MUST match: same
 * field order, same types, same ABI (verified by the C++ caller casting
 * NativeTierSpec* → nros_native_tier_spec_t* → this type). Gated by
 * `check-ffi-struct-mirrors`, which compares this declaration to main.h's
 * field by field (the `nros_tier_setup_fn_t` typedef being the one
 * legitimate spelling difference). */
typedef int32_t (*nros_tier_setup_fn_t)(void* executor);

typedef struct {
    const char* name;
    const char* const* groups;
    size_t n_groups;
    int64_t priority;
    size_t stack_bytes;
    uint64_t spin_period_us;
    nros_tier_setup_fn_t setup;
    /* RFC-0052 W2 — appended (ABI append-only): CPU pin + 1 (0 = unpinned)
     * and the ThreadX preemption threshold (-1 = unset; bake-validated
     * ThreadX-only, so this mirror never consumes it). Keep the offset
     * comment above in sync when appending further. */
    uint32_t core_plus1;
    int64_t preempt_threshold;
    /* phase-296 W5.7 — appended: RTOS-agnostic real-time policy (NULL/0 =
     * unset). Not consumed by this mirror yet (NuttX kernel-native sporadic
     * is a follow-up); rides for layout parity. */
    const char* tier_class;
    uint64_t period_us;
    uint64_t budget_us;
    uint64_t deadline_us;
    const char* deadline_policy;
} nros_tier_spec_t;

/* --- Per-tier thread context ---
 *
 * Heap-allocated by the spawning thread before pthread_create; lives for the
 * firmware lifetime (the spawned thread never returns). executor_storage is a
 * separate heap block passed to nros_cpp_executor_open_over_session. */
typedef struct {
    void* session_handle;
    uint32_t domain_id;
    const char* const* groups;
    size_t n_groups;
    uint64_t spin_period_us;
    nros_tier_setup_fn_t setup;
    void* executor_storage;
    /* issue #144 — chained spawn tail: the tiers still to bring up AFTER this
     * one. This thread spawns rest[0] (carrying rest[1..]) only after its own
     * setup returns, so no two setups overlap on the shared session. */
    const nros_tier_spec_t* rest;
    size_t n_rest;
    /* phase-296 W5.9 — tier identity + sporadic policy, carried so the tier
     * thread can self-apply SCHED_SPORADIC (mirrors the zephyr ctx append). */
    const char* name;
    const char* tier_class;
    uint64_t budget_us;
    uint64_t period_us;
    int64_t priority;
    /* phase-296 W5.11 — placement dim: CPU pin + 1 (0 = unpinned), carried so
     * the spawned tier thread can self-apply its core affinity. */
    uint32_t core_plus1;
} nros_nuttx_tier_ctx_t;

/* Forward decl — nuttx_tier_thread and nuttx_spawn_next_tier are mutually
 * recursive (each tier's thread spawns the next tier via this helper). */
static int nuttx_spawn_next_tier(void* session_handle, uint8_t domain_id,
                                 const nros_tier_spec_t* remaining, size_t n_remaining);

/* Minimum spin delay: 1 ms. */
#define SPIN_PERIOD_FLOOR_MS 1u

/* Clamp a raw tier priority (int64 from the tier spec) to the SCHED_FIFO range
 * NuttX accepts, so pthread_attr_setschedparam / pthread_setschedparam never
 * reject it. */
static int nuttx_clamp_priority(int64_t p) {
    int lo = sched_get_priority_min(SCHED_FIFO);
    int hi = sched_get_priority_max(SCHED_FIFO);
    if (p < (int64_t)lo) {
        return lo;
    }
    if (p > (int64_t)hi) {
        return hi;
    }
    return (int)p;
}

/* phase-296 W5.9 — NuttX kernel-native SPORADIC SERVER (the budget dim's
 * first `Native` realization, RFC-0052). Applies SCHED_SPORADIC on the
 * CALLING thread when the tier is real-time with BOTH a budget and a
 * replenishment period: runs at the tier priority while `budget_us` of CPU
 * remains in each `period_us` window, dropping to the low background
 * priority when exhausted (POSIX sporadic server; NuttX implements it when
 * CONFIG_SCHED_SPORADIC=y). The marker prints ONLY when the kernel actually
 * accepted the policy — an image without the config (or a kernel rejection)
 * falls back to the already-applied SCHED_FIFO, loudly, and the executor's
 * cooperative Sporadic SchedContext (W3a) remains the enforcement.
 * Non-static: the Rust `nros-board-nuttx` run_tiers externs and self-applies
 * through this same helper (one implementation, one marker). The printf
 * literal MUST match `nros_tests::output::NUTTX_SPORADIC_MARKER`. */
int nros_nuttx_apply_current_sporadic(const char* name, const char* tier_class, uint64_t budget_us,
                                      uint64_t period_us, int64_t priority);
int nros_nuttx_apply_current_sporadic(const char* name, const char* tier_class, uint64_t budget_us,
                                      uint64_t period_us, int64_t priority) {
    if (tier_class == NULL || strcmp(tier_class, "real_time") != 0 || budget_us == 0u ||
        period_us == 0u) {
        return 0;
    }
#ifdef CONFIG_SCHED_SPORADIC
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = nuttx_clamp_priority(priority);
    sp.sched_ss_low_priority = sched_get_priority_min(SCHED_FIFO);
    /* issue 0736 — the kernel's configured maximum, NOT 1.
     *
     * `sched_ss_max_repl` bounds how many replenishments may be IN FLIGHT.
     * NuttX allocates one per suspend/resume of the thread (sched_sporadic.c
     * `sporadic_alloc_repl`), and an executor tier suspends every spin — 1000us
     * here. With one slot, the second suspend finds none free, and the kernel's
     * documented fall-back is to DO NOTHING:
     *
     *     serr("ERROR: Failed to allocate timer, nrepls=%d\n", ...);
     *     // "Doing nothing is our fall-back behavior and that is not a
     *     //  failure from the standpoint of higher level logic."
     *
     * The consumed budget is then never replenished, so the thread stays at
     * `sched_ss_low_priority` — which is the FLOOR, below every other tier —
     * for the rest of its life. Measured: the 10ms `high` tier published at
     * ~1 Hz while the 100ms `low` tier ran at ~10 Hz, an inversion of ~70x.
     *
     * 1 is only correct for a thread that runs exactly once per period and is
     * never preempted inside it, which no spin-loop tier is. The kernel's own
     * bound is the right ceiling to ask for: it is what this build's config
     * sized the array to, and `sporadic_alloc_repl` DEBUGASSERTs against
     * exactly this value.
     *
     * Latent until #636 — the boot tier's path does not apply sporadic at all,
     * and `high` WAS the boot tier until `boot_tier_index` made the least
     * urgent tier the session owner. Moving it to the spawned path reached
     * this line for the first time.
     *
     * HONEST SCOPE: this is NOT the cause of issue 0736, which is what found
     * it. Measured on nuttx-arm/rust: 1 -> CONFIG (3) left the symptom bit for
     * bit unchanged (ctrl 2, telem 21), and disabling SCHED_SPORADIC outright
     * did not fix it either (ctrl 5, telem 13 — still inverted). 0736 turned
     * out to be the tier's TIMER under-firing, not its scheduling. The line is
     * corrected anyway because 1 is wrong on its own terms.
     */
    sp.sched_ss_max_repl = CONFIG_SCHED_SPORADIC_MAXREPL;
    sp.sched_ss_repl_period.tv_sec = (time_t)(period_us / 1000000u);
    sp.sched_ss_repl_period.tv_nsec = (long)(period_us % 1000000u) * 1000L;
    sp.sched_ss_init_budget.tv_sec = (time_t)(budget_us / 1000000u);
    sp.sched_ss_init_budget.tv_nsec = (long)(budget_us % 1000000u) * 1000L;
    int rc = pthread_setschedparam(pthread_self(), SCHED_SPORADIC, &sp);
    if (rc == 0) {
        printf("nros: sporadic budget set tier=`%s` %lluus/%lluus\n", (name != NULL) ? name : "?",
               (unsigned long long)budget_us, (unsigned long long)period_us);
        return 1;
    }
    printf("nros: sporadic budget FAILED tier=`%s` rc=%d — tier stays SCHED_FIFO "
           "(executor Sporadic SchedContext is the enforcement)\n",
           (name != NULL) ? name : "?", rc);
#else
    /* Fail-loud (RFC-0052): the tier DECLARED a sporadic budget but this
     * kernel was built without CONFIG_SCHED_SPORADIC — the declared policy
     * cannot be honored natively. The executor's cooperative Sporadic
     * SchedContext (W3a) is the enforcement; say so once per tier. */
    printf("nros: sporadic budget declared for tier=`%s` but kernel lacks "
           "CONFIG_SCHED_SPORADIC — executor SchedContext only\n",
           (name != NULL) ? name : "?");
    (void)priority;
#endif
    return 0;
}

/* phase-296 W5.11 — NuttX SMP core affinity (the placement dim's `Native`
 * realization, RFC-0052). Pins the CALLING thread to `core_plus1 - 1` when a
 * tier declares a `core` (core_plus1 > 0 — the emit_c/macro `core + 1` encoding,
 * 0 = unpinned). The marker prints ONLY when the kernel accepted the pin; a
 * uniprocessor build (no CONFIG_SMP) or a rejection falls back LOUDLY — the
 * placement is NEVER silently dropped (the prior behavior: the ABI carried
 * `core_plus1` but no consumer applied it). Non-static: the Rust
 * `nros-board-nuttx` run_tiers externs and self-applies through this same
 * helper (one implementation, one marker). The printf literals MUST match
 * `nros_tests::output::NUTTX_CORE_PIN_MARKER` / `_FALLBACK_MARKER`. */
int nros_nuttx_apply_current_affinity(const char* name, uint32_t core_plus1);
int nros_nuttx_apply_current_affinity(const char* name, uint32_t core_plus1) {
    if (core_plus1 == 0u) {
        return 0; /* unpinned */
    }
    int cpu = (int)(core_plus1 - 1u);
#ifdef CONFIG_SMP
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (rc == 0) {
        printf("nros: core pin tier=`%s` cpu=%d\n", (name != NULL) ? name : "?", cpu);
        return 1;
    }
    printf("nros: core pin FAILED tier=`%s` cpu=%d rc=%d — tier runs unpinned\n",
           (name != NULL) ? name : "?", cpu, rc);
#else
    /* Fail-loud (RFC-0052): the tier DECLARED a `core` but this kernel was
     * built without CONFIG_SMP — no affinity API to honor it. */
    printf("nros: core pin FAILED tier=`%s` cpu=%d — kernel lacks CONFIG_SMP, "
           "tier runs unpinned\n",
           (name != NULL) ? name : "?", cpu);
#endif
    return 0;
}

/* issue 0750 — report where the tier ACTUALLY runs, not just that the kernel
 * took the mask.
 *
 * The pin markers above say `pthread_setaffinity_np` returned 0. On a
 * uniprocessor image that is true and uninformative: cpu 0 is the only CPU the
 * thread could be on, so an accept there proves the call is well-formed and
 * nothing about placement. This is the line that distinguishes "the kernel
 * accepted the request" from "the kernel honoured it" — a kernel that takes the
 * mask and then schedules elsewhere fails here and passes there.
 *
 * Mirrors `nros-board-zephyr`'s `nros: core pin observed` (both the C and Rust
 * arms). The wording and shape are deliberately identical so one test marker
 * serves both boards and the two cannot drift.
 *
 * Guarded on CONFIG_SMP for the same reason Zephyr's is: `sched_getcpu()` is
 * declared in the non-SMP export too and would answer a constant 0 there, which
 * is a fabricated placement report. Silent beats invented — the cell that reads
 * this treats "absent" as failure rather than tolerating a second arm. */
void nros_nuttx_report_observed_cpu(const char* name);
void nros_nuttx_report_observed_cpu(const char* name) {
#ifdef CONFIG_SMP
    int observed = sched_getcpu();
    if (observed >= 0) {
        printf("nros: core pin observed tier=`%s` running_on=%d\n", (name != NULL) ? name : "?",
               observed);
    }
#else
    (void)name;
#endif
}

/* phase-302 W3 (issue 0263) — adopt the tier's declared SCHED_FIFO priority
 * on the CALLING thread. The C arm applies priority at pthread_create time;
 * the Rust arm spawns via std::thread (no priority attr) and self-applies
 * through this helper at tier entry — one implementation, one marker. The
 * printf literal MUST match `nros_tests::output::NUTTX_TIER_PRIORITY_MARKER`. */
int nros_nuttx_apply_current_priority(const char* name, uint32_t priority);
int nros_nuttx_apply_current_priority(const char* name, uint32_t priority) {
    if (priority == 0u) {
        return 0; /* undeclared — keep inherited priority */
    }
    struct sched_param sp;
    sp.sched_priority = (int)priority;
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc == 0) {
        printf("nros: tier priority set tier=`%s` prio=%d\n", (name != NULL) ? name : "?",
               (int)priority);
        return 1;
    }
    printf("nros: tier priority FAILED tier=`%s` prio=%d rc=%d — tier runs at inherited "
           "priority\n",
           (name != NULL) ? name : "?", (int)priority, rc);
    return 0;
}

/* phase-364 W3 — `nros_nuttx_spawn_tier` is GONE.
 *
 * phase-359 W7 added it because the platform ABI had no portable way to ask for
 * a stack size, and building a `pthread_attr_t` from Rust is issue 0570. W3
 * gave the ABI `nros_platform_task_attr_t`, so the Rust arm calls
 * `nros_platform_task_init` directly and this shim has nothing left to do. The
 * `pthread_attr_t` layout that must not be mirrored stays inside the POSIX port
 * — which is where it always belonged, and which NuttX runs. */

/* nuttx_tier_thread — body of each non-boot tier thread.
 *
 * Opens a borrowed executor over the shared session, gates it to the tier's
 * callback groups, calls the tier's setup function, spawns the next tier
 * (issue #144), then spins forever at the tier's declared period. On failure:
 * parks (never returns — the boot thread is the session owner and must outlive
 * all borrowed executors). Signature is `void* (*)(void*)` to match
 * pthread_create's entry type. */
/* issue 0736 — REPORT when the kernel sporadic server is throttling this tier.
 *
 * The issue's own conclusion is that the residual rate shortfall isolates to
 * NuttX's `SCHED_SPORADIC` server (4/4 FAIL with `apply_tier_sporadic`, 3/3 PASS
 * without) and that "the kernel side has no equivalent" of the executor's
 * throttle report: it throttles silently and nothing in the image can see it.
 * This closes that gap.
 *
 * DIRECTLY OBSERVED, not inferred. The POSIX sporadic server demotes a thread to
 * `sched_ss_low_priority` when its budget is exhausted, and this port sets that
 * floor to `sched_get_priority_min(SCHED_FIFO)`. A thread running at the floor
 * when it declared a higher priority IS a thread the kernel has throttled — and
 * a demoted thread still runs, so it can sample itself. (`sporadic_s.suspended`
 * would be the other signal and is useless here: a suspended thread cannot
 * observe itself.)
 *
 * That distinction matters on this issue specifically, which has retracted one
 * conclusion already for resting on an experiment with no control. This asks the
 * kernel what priority the thread is at, and reports the answer.
 *
 * Reported ONCE per tier. A tier that dips to the floor for one period and one
 * that never recovers are the same first message; the second is visible as the
 * message arriving early and the tier's rate never improving. Repeating it every
 * spin would bury the run in output at 1000 Hz.
 */
static void nros_nuttx_report_sporadic_throttle(const char* name, int declared_priority,
                                                int* already_reported) {
#ifdef CONFIG_SCHED_SPORADIC
    if (*already_reported) {
        return;
    }
    struct sched_param now;
    if (sched_getparam(0, &now) != 0) {
        return;
    }
    int floor_prio = sched_get_priority_min(SCHED_FIFO);
    if (now.sched_priority <= floor_prio && declared_priority > floor_prio) {
        *already_reported = 1;
        printf("nros: sporadic budget throttled by the KERNEL tier=`%s` prio=%d (declared %d) — "
               "the SCHED_SPORADIC budget is exhausted and this tier is running at the floor\n",
               (name != NULL) ? name : "?", now.sched_priority, declared_priority);
    }
#else
    (void)name;
    (void)declared_priority;
    (void)already_reported;
#endif
}

static void* nuttx_tier_thread(void* arg) {
    nros_nuttx_tier_ctx_t* ctx = (nros_nuttx_tier_ctx_t*)arg;

    /* issue 0636 coverage — REPORT the priority this tier was born with.
     *
     * `nros_nuttx_apply_current_priority` existed with the right marker and no
     * caller: the C arm set priorities at `pthread_create` time and printed
     * nothing, so the whole language arm was invisible to the `TierPriority`
     * dim. Its own doc comment says the helper is "one implementation, one
     * marker" shared with the Rust arm, and the Rust arm calls it at tier
     * entry — so does this one now.
     *
     * Re-applying the priority the attribute already set is deliberate and
     * cheap: it is the same value, so the call is a no-op in effect, and it is
     * the only point where this thread can say what it actually got. #579's
     * rule is that every DECLARING tier prints its own marker; a tier that was
     * configured correctly and says so is what separates that from a tier
     * nobody scheduled. */
    (void)nros_nuttx_apply_current_priority(ctx->name, ctx->priority);

    /* phase-296 W5.9 — upgrade this tier thread to the kernel sporadic
     * server when its policy declares budget+period (else it keeps the
     * SCHED_FIFO priority set at pthread_create). */
    (void)nros_nuttx_apply_current_sporadic(ctx->name, ctx->tier_class, ctx->budget_us,
                                            ctx->period_us, ctx->priority);
    /* phase-296 W5.11 — placement dim: pin to the declared core (fail-loud). */
    (void)nros_nuttx_apply_current_affinity(ctx->name, ctx->core_plus1);
    /* issue 0750 — and say where it LANDED, after the pin and before any work.
     * Only the spawned tiers report: the boot tier is already running when the
     * mask is applied, so its placement says more about when it was asked than
     * about whether the pin worked. */
    nros_nuttx_report_observed_cpu(ctx->name);

    /* Open a borrowed executor that shares the primary session. The primary
     * executor (boot thread) must outlive this thread — the boot spin loop runs
     * forever, enforcing this. */
    int rc = nros_cpp_executor_open_over_session(ctx->session_handle, "tier_node", ctx->domain_id,
                                                 ctx->executor_storage);
    if (rc != 0) {
        /* Cannot open the borrowed executor; park forever (boot thread
         * continues). NOTE (issue #144): the spawn of the next tier sits AFTER
         * this tier's setup, so failing here HALTS the chain — ctx->rest will
         * not start. Intentional (a degraded deploy), but a fault-isolation
         * change from the pre-#144 loop-spawn. */
        for (;;) {
            usleep(1000000u);
        }
    }

    /* Gate this executor to its tier's callback groups. */
    if (ctx->n_groups > 0 && ctx->groups != NULL) {
        nros_cpp_executor_set_active_groups(ctx->executor_storage, ctx->groups, ctx->n_groups);
    }

    /* Run the tier's node-setup function. */
    if (ctx->setup != NULL) {
        rc = ctx->setup(ctx->executor_storage);
        if (rc != 0) {
            /* Setup failed; close the borrowed executor and park. As with the
             * open failure above, this HALTS the chain (issue #144). */
            nros_cpp_fini(ctx->executor_storage);
            for (;;) {
                usleep(1000000u);
            }
        }
    }

    /* issue #144 — this tier's setup is done, so bringing up the next tier can
     * no longer race our declares: spawn rest[0] (carrying rest[1..]). A failed
     * DOWNSTREAM spawn must NOT stop this tier spinning its own work, so ignore
     * the return (nuttx_spawn_next_tier frees what it allocated on failure). */
    (void)nuttx_spawn_next_tier(ctx->session_handle, (uint8_t)ctx->domain_id, ctx->rest,
                                ctx->n_rest);

    /* Spin loop. Pass the tier period as the spin_once timeout — a BLOCKING
     * read drives the shared session's TX/handshake from the spin path and
     * yields the CPU cooperatively (on NuttX `zpico_spin_once` paces with
     * z_sleep_ms; the preemptive scheduler releases the CPU to the zenoh-pico
     * read/lease tasks). Mirrors the Rust nuttx run_tiers tier loop. */
    uint32_t period_ms = (uint32_t)(ctx->spin_period_us / 1000u);
    if (period_ms < SPIN_PERIOD_FLOOR_MS) {
        period_ms = SPIN_PERIOD_FLOOR_MS;
    }
    /* issue 0636 option 3 — every iteration reaches a scheduling point. The
     * executor's own wait is skipped whenever a wake already fired, so under
     * sustained traffic this loop would otherwise never block, and under
     * SCHED_FIFO a thread that never blocks never lets a lower-priority tier
     * run. The gap costs nothing while the spins do block. */
    uint64_t gap_state = 0;
    /* issue 0736 — see nros_nuttx_report_sporadic_throttle. */
    int throttle_reported = 0;
    int declared_prio = nuttx_clamp_priority(ctx->priority);
    for (;;) {
        uint64_t iter_start_ns = nros_platform_clock_ns();
        nros_cpp_spin_once(ctx->executor_storage, (int32_t)period_ms);
        gap_state = nros_tier_spin_gap_step(gap_state, iter_start_ns, nros_platform_clock_ns(),
                                            ctx->spin_period_us);
        nros_nuttx_report_sporadic_throttle(ctx->name, declared_prio, &throttle_reported);
    }

    /* Unreachable — the spin loop never exits; satisfies the non-void return. */
    return NULL;
}

/* nuttx_spawn_next_tier — issue #144 chained tier spawn.
 *
 * Spawns exactly ONE pthread for remaining[0], handing it remaining[1..] as its
 * own `rest` so the chain continues once its setup completes. Empty `remaining`
 * → nothing left, return 0. Serializing spawns behind each setup guarantees no
 * two setup() (entity declare) calls run concurrently on the shared zenoh-pico
 * session — the interest-handshake race that silently closes a losing
 * publisher's write filter.
 *
 * On any alloc/create failure, frees what IT allocated and returns -1. It does
 * NOT touch the caller's storage. */
static int nuttx_spawn_next_tier(void* session_handle, uint8_t domain_id,
                                 const nros_tier_spec_t* remaining, size_t n_remaining) {
    if (n_remaining == 0u) {
        return 0;
    }
    const nros_tier_spec_t* t = &remaining[0];

    /* Allocate executor storage for this tier. */
    void* tier_exec = nros_platform_alloc(NROS_NUTTX_EXECUTOR_STORAGE_BYTES);
    if (tier_exec == NULL) {
        return -1;
    }
    memset(tier_exec, 0, NROS_NUTTX_EXECUTOR_STORAGE_BYTES);

    /* Allocate the tier thread context (lives for firmware lifetime). */
    nros_nuttx_tier_ctx_t* ctx =
        (nros_nuttx_tier_ctx_t*)nros_platform_alloc(sizeof(nros_nuttx_tier_ctx_t));
    if (ctx == NULL) {
        nros_platform_dealloc(tier_exec);
        return -1;
    }

    ctx->session_handle = session_handle;
    ctx->domain_id = (uint32_t)domain_id;
    ctx->groups = t->groups;
    ctx->n_groups = t->n_groups;
    ctx->spin_period_us = t->spin_period_us;
    ctx->setup = t->setup;
    ctx->executor_storage = tier_exec;
    /* Chain tail: this thread will spawn remaining[1] after its own setup. */
    ctx->rest = remaining + 1;
    ctx->n_rest = n_remaining - 1u;
    ctx->name = t->name;
    ctx->tier_class = t->tier_class;
    ctx->budget_us = t->budget_us;
    ctx->period_us = t->period_us;
    ctx->priority = t->priority;
    ctx->core_plus1 = t->core_plus1;

    /* Build the tier pthread attributes: detached (never joined — the tier
     * thread spins forever), an explicit stack size (tier.stack_bytes, else the
     * 16 KiB default), and SCHED_FIFO at the tier's RAW NuttX priority with
     * PTHREAD_EXPLICIT_SCHED so the policy/priority actually apply (default is
     * PTHREAD_INHERIT_SCHED → the creator's params). */
    pthread_attr_t attr;
    int arc = pthread_attr_init(&attr);
    if (arc != 0) {
        nros_platform_dealloc(ctx);
        nros_platform_dealloc(tier_exec);
        return -1;
    }
    (void)pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    size_t stack_bytes = (t->stack_bytes > 0u) ? t->stack_bytes : NROS_NUTTX_TIER_STACK_BYTES;
    (void)pthread_attr_setstacksize(&attr, stack_bytes);

    (void)pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    (void)pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = nuttx_clamp_priority(t->priority);
    (void)pthread_attr_setschedparam(&attr, &sp);

    pthread_t tid;
    int ret = pthread_create(&tid, &attr, nuttx_tier_thread, ctx);
    (void)pthread_attr_destroy(&attr);
    if (ret != 0) {
        nros_platform_dealloc(ctx);
        nros_platform_dealloc(tier_exec);
        return -1;
    }
    return 0;
}

/* nros_board_nuttx_run_tiers — phase-281 W3 (RFC-0015 Model 1 embedded).
 *
 * Called from NuttxBoard::run_tiers (main.hpp) which is called from the
 * generated nros_app_main. By this point: the NuttX kernel is running, eth0 is
 * up (the board FFI main brought up virtio-net before app_main — phase-280), and
 * we are executing on the app_main thread.
 *
 * The `tiers` array is laid out identically to nros_native_tier_spec_t (the
 * caller casts NativeTierSpec* → nros_native_tier_spec_t* → nros_tier_spec_t*).
 *
 * `locator`      — zenoh connect endpoint (compile-time NROS_ENTRY_LOCATOR)
 * `domain_id`    — ROS domain ID (compile-time NROS_ENTRY_DOMAIN_ID)
 * `session_name` — primary session / node name; NULL or empty → "node"
 * `tiers`        — tier-spec array, highest-priority-first (codegen order)
 * `n_tiers`      — number of tiers (>= 1)
 *
 * Returns: normally never returns (the boot tier spins forever). Returns a
 * non-zero error code only if the primary session open or a spawn fails. */
int32_t nros_board_nuttx_run_tiers(const char* locator, uint8_t domain_id, const char* session_name,
                                   const nros_tier_spec_t* tiers, size_t n_tiers) {
    if (tiers == NULL || n_tiers == 0) {
        return -3; /* NROS_CPP_RET_INVALID_ARGUMENT */
    }

    /* Weak network-readiness gate (no-op on the canonical NuttX path — eth0 is
     * already up from the board FFI main; a board/app may provide a strong
     * override). */
    nros_board_network_wait();

    /* --- Open the primary (owning) executor on the boot thread --- */
    const char* sn = (session_name != NULL && session_name[0] != '\0') ? session_name : "node";

    /* Allocate executor storage from the NuttX heap (aligned). */
    void* boot_storage = nros_platform_alloc(NROS_NUTTX_EXECUTOR_STORAGE_BYTES);
    if (boot_storage == NULL) {
        return -1; /* NROS_CPP_RET_ERROR */
    }
    memset(boot_storage, 0, NROS_NUTTX_EXECUTOR_STORAGE_BYTES);

    int rc = nros_cpp_init(locator, domain_id, sn, NULL, boot_storage);
    if (rc != 0) {
        nros_platform_dealloc(boot_storage);
        return (int32_t)rc;
    }

    /* Retrieve the session handle so non-boot tiers can borrow it. The handle
     * remains valid as long as boot_storage lives (it lives forever — the boot
     * spin loop never returns). */
    void* session_handle = nros_cpp_executor_session_handle(boot_storage);

    /* --- Choose the boot tier: the LEAST urgent one (issue 0636) --- */
    /*
     * This used to be `&tiers[0]`, which the emitter documents as the MOST
     * urgent tier ("sorted highest-priority-first", nros/main.hpp). The boot
     * thread owns the session and spins forever, so making it outrank every
     * tier it spawned starves them: under SCHED_FIFO on a uniprocessor guest a
     * lower-priority peer runs only when the holder BLOCKS, and a spin loop
     * does not reliably block. `17666723d` removed that arrangement from the
     * Rust arm (`nros_platform::boot_tier_index`); this file kept it, so the
     * two language arms disagreed about which tier owns the session on one
     * board.
     *
     * The array is sorted DESCENDING by raw priority and NuttX is
     * bigger-is-more-urgent, so the least urgent tier is the LAST element and
     * the remaining tiers stay CONTIGUOUS — which is what lets the existing
     * chain-spawn walk them unchanged.
     *
     * That relies on the emitter's ordering, so it is CHECKED rather than
     * assumed: a table that is not non-increasing means the contract changed,
     * and the failure it would otherwise produce is silent starvation seconds
     * later on one platform. Report and continue with index 0 — the behaviour
     * before this change — rather than pick a tier from an ordering nobody
     * guaranteed.
     */
    size_t boot_idx = n_tiers - 1u;
    for (size_t i = 1u; i < n_tiers; ++i) {
        if (tiers[i].priority > tiers[i - 1u].priority) {
            printf("nros: tier table is not sorted highest-priority-first — "
                   "boot tier falls back to index 0 (issue 0636)\n");
            boot_idx = 0u;
            break;
        }
    }
    const nros_tier_spec_t* boot = &tiers[boot_idx];
    /* The tiers the boot thread must spawn: everything except `boot`. Both
     * arrangements leave a contiguous run. */
    const nros_tier_spec_t* rest_first = (boot_idx == 0u) ? &tiers[1] : &tiers[0];
    const size_t n_rest = n_tiers - 1u;

    /* issue #144 — boot setup runs BEFORE any tier spawn: concurrent entity
     * declares from two threads race the zenoh-pico interest handshake, and the
     * losing publisher's write filter stays closed (every put silently
     * dropped). Running boot's declares first, then CHAINING the remaining
     * spawns, makes setup order total (boot, t1, t2, …) so no two declares
     * overlap. */

    /* Gate the boot executor to its tier's callback groups. */
    if (boot->n_groups > 0 && boot->groups != NULL) {
        nros_cpp_executor_set_active_groups(boot_storage, boot->groups, boot->n_groups);
    }

    /* Boot tier node setup. */
    if (boot->setup != NULL) {
        rc = boot->setup(boot_storage);
        if (rc != 0) {
            nros_cpp_fini(boot_storage);
            nros_platform_dealloc(boot_storage);
            return (int32_t)rc;
        }
    }

    /* The boot thread runs its chosen tier itself — adopt that tier's declared
     * RAW priority
     * (the spawned tiers already got theirs at pthread_create; without this the
     * boot tier keeps the app_main-thread default and the declared tier QoS
     * would not hold for it).
     *
     * issue 0636 — this runs BEFORE the spawn below. NOT because it fixes the
     * cell: with the harness short read repaired (`wait_for_output_each`, which
     * is what actually failed here) the cell is 12/12 with this block back
     * AFTER the spawn. It stays here because the window it closes is real, and
     * because a dim applied behind a spawn that preempts you is only ever
     * correct by luck. `boot` is the LEAST urgent tier, so every tier this thread
     * spawns outranks it: on the realtime-cpp table (`high` 110, `low` 100,
     * `low` == boot) the child is created at SCHED_FIFO 110 and preempts this
     * thread the instant `pthread_create` returns, at the default app_main
     * priority. Anything sitting after the spawn therefore runs only once the
     * child first blocks. That is a real window; it was NOT the reported
     * failure, which was the test reader returning at the first marker and
     * killing QEMU (so whichever tier printed SECOND read as dropped, and
     * moving this block merely swapped which one that was). Nothing here needs the children to
     * exist, and no other tier thread exists yet to be starved by a
     * self-demotion, so the dims are applied while this thread still owns the
     * CPU. #144's ordering is untouched: boot's DECLARES already ran above.
     *
     * The Rust arm does the mirror of this and is green (67/67) for the mirror
     * reason: it spawns EVERY tier from this thread in a loop, so it must keep
     * its inherited priority across the whole loop or it never gets the CPU
     * back to finish spawning. This arm chain-spawns — exactly one create, and
     * tier N brings up tier N+1 — so there is no later spawn to protect. */
    /* issue 0636 coverage — through the shared helper, so the boot tier prints
     * its marker like every other declaring tier (#579). This was an inline
     * `pthread_setschedparam` that reported nothing, which is how the boot
     * tier could be the ONLY tier whose priority nobody could confirm from the
     * console — on the arm where the boot tier is also the one this issue kept
     * finding starved. The helper clamps and reports; it no-ops on priority 0
     * (undeclared), which is the same "keep what was inherited" the inline
     * version produced. */
    (void)nros_nuttx_apply_current_priority(boot->name,
                                            (uint32_t)nuttx_clamp_priority(boot->priority));
    /* phase-296 W5.9 — boot tier upgrades to the kernel sporadic server too,
     * when declared (overrides the FIFO adopt above). */
    (void)nros_nuttx_apply_current_sporadic(boot->name, boot->tier_class, boot->budget_us,
                                            boot->period_us, boot->priority);
    /* phase-296 W5.11 — placement dim: pin the boot tier too (fail-loud). */
    (void)nros_nuttx_apply_current_affinity(boot->name, boot->core_plus1);

    /* --- Kick off the chained spawn (tiers[1] carrying tiers[2..]) --- */
    /* A boot-side spawn failure is fatal: tear down boot_storage (which the
     * helper never touches) and return error. Downstream tier threads handle
     * their own spawn failures by parking + continuing to spin. */
    int src = nuttx_spawn_next_tier(session_handle, domain_id, rest_first, n_rest);
    if (src != 0) {
        nros_cpp_fini(boot_storage);
        nros_platform_dealloc(boot_storage);
        return -1;
    }


    /* Boot tier spin loop — runs forever. Blocking-read spin_once (period as
     * timeout) so the boot session's zenoh handshake is driven from the spin
     * path and the CPU yields cooperatively (mirrors the Rust nuttx run_tiers
     * boot loop). */
    uint32_t period_ms = (uint32_t)(boot->spin_period_us / 1000u);
    if (period_ms < SPIN_PERIOD_FLOOR_MS) {
        period_ms = SPIN_PERIOD_FLOOR_MS;
    }
    /* issue 0636 option 3 — every iteration reaches a scheduling point. The
     * executor's own wait is skipped whenever a wake already fired, so under
     * sustained traffic this loop would otherwise never block, and under
     * SCHED_FIFO a thread that never blocks never lets a lower-priority tier
     * run. The gap costs nothing while the spins do block. */
    uint64_t gap_state = 0;
    for (;;) {
        uint64_t iter_start_ns = nros_platform_clock_ns();
        nros_cpp_spin_once(boot_storage, (int32_t)period_ms);
        gap_state = nros_tier_spin_gap_step(gap_state, iter_start_ns, nros_platform_clock_ns(),
                                            boot->spin_period_us);
    }

    /* Unreachable — satisfies the compiler. */
    nros_cpp_fini(boot_storage);
    nros_platform_dealloc(boot_storage);
    return 0;
}
