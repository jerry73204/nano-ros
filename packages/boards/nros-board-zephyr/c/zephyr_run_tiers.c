/*
 * zephyr_run_tiers.c — phase-281 W3a: C++ embedded multi-tier entry for Zephyr.
 *
 * Implements `nros_board_zephyr_run_tiers`, the Zephyr analog of the FreeRTOS
 * `nros_board_freertos_run_tiers` (and the native `nros_board_native_run_tiers`):
 * opens ONE RMW session on the calling thread (the Zephyr `main()` thread — an
 * already-running post-init thread), runs the boot tier's setup, then
 * CHAIN-spawns the remaining tiers (each with a borrowed executor sharing the
 * one session) so their setups serialize (issue #144), and runs the boot tier
 * on the caller.
 *
 * Zephyr deltas vs the FreeRTOS mirror:
 *   - non-boot tiers spawn via the `nros_zephyr_tier_task_create` k_thread shim
 *     (`zephyr/nros_platform_zephyr_shims.c`), NOT xTaskCreate; the shim owns a
 *     static tier stack pool + RAW Zephyr priorities;
 *   - the boot tier adopts its declared RAW priority on the caller thread via
 *     `nros_zephyr_set_current_priority` (the spawned tiers get theirs at
 *     `k_thread_create`);
 *   - Zephyr owns boot + networking (`CONFIG_NET_CONFIG_AUTO_INIT`), so there is
 *     no lwIP bring-up here — just the weak `nros_board_network_wait()` gate
 *     (mirrors `ZephyrBoard::run_components` in main.hpp), not the FreeRTOS
 *     network path.
 *
 * Called from `ZephyrBoard::run_tiers` (main.hpp) via the generated `int
 * main(void)` (the Zephyr kernel calls `main` directly — no `nros_app_main`).
 *
 * phase-281 W3a / RFC-0015 §5.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* --- Zephyr k_thread tier shim (zephyr/nros_platform_zephyr_shims.c) ---
 *
 * `nros_zephyr_tier_task_create`: spawn one tier task on a static pool thread
 * at the RAW Zephyr priority (cooperative if negative). Returns 0 on success,
 * -1 when the pool (NROS_ZEPHYR_MAX_TIERS) is exhausted. The `entry(arg)`
 * signature is `void* (*)(void*)`.
 *
 * `nros_zephyr_set_current_priority`: adopt a RAW Zephyr priority on the CALLING
 * thread (the boot thread runs tiers[0] itself).
 *
 * `nros_zephyr_msleep`: real-symbol wrapper around `k_msleep` for the idle /
 * park loops (the k_msleep inline has no exported symbol). */
/* issue 0655 — `core_plus1` (0 = unpinned) makes the shim create the thread
 * SUSPENDED and pin it before starting, the only window in which Zephyr
 * accepts a cpu mask. `pin_rc` returns the kernel's own code so the marker
 * text stays here, in lockstep with nros_tests::output::ZEPHYR_CORE_PIN_*. */
/* issue 0260 — the CPU the CALLING thread is observed on, or
 * NROS_ZEPHYR_CPU_UNKNOWN (0xFFFFFFFF) when the image cannot say. SMP-only by
 * construction: `arch_proc_id()` is declared inside `#ifdef CONFIG_SMP`, and
 * the posix arch (native_sim) does not provide it at all. */
extern uint32_t nros_zephyr_current_cpu(void);
/* issue 0758 W4 — see zephyr/nros_platform_zephyr_shims.c. */
extern void nros_zephyr_epoch_acquire_configured(void);

/* phase-8 W4 — callback tracing. The shim is an unconditional symbol whose
 * body is `#ifdef CONFIG_TRACING_CTF`; the setter is an unconditional export
 * from nros-c whose body is feature-gated. So this pair links in EVERY
 * configuration and costs a single stored null pointer when either half is
 * absent — which is what lets the call below stay unguarded. */
extern void nros_zephyr_trace_marker(uint32_t marker_id, uint32_t arg);
extern void nros_set_trace_sink(void (*sink)(uint32_t, uint32_t));

extern int nros_zephyr_tier_task_create(void* (*entry)(void*), void* arg, int32_t priority,
                                        const char* name, size_t stack_bytes,
                                        uint32_t core_plus1, int* pin_rc);
extern void nros_zephyr_set_current_priority(int32_t priority);
extern int32_t nros_zephyr_msleep(int32_t ms);
/* Phase 110.D shim — pin the CALLING thread to a CPU (`k_thread_cpu_pin`).
 * Returns 0 on success, -ENOSYS when the image lacks CONFIG_SCHED_CPU_MASK,
 * else the kernel error. issue 0655: for a STARTED thread that error is always
 * -EINVAL, so this is the BOOT-tier path only — spawned tiers are pinned by
 * `nros_zephyr_tier_task_create` before their thread starts. */
extern int nros_zephyr_thread_cpu_pin(int cpu);
/* phase-296 W5.5 shim — apply an earliest-deadline (µs) on the CALLING
 * thread (`k_thread_deadline_set`, µs→cycles). Returns 1 when the kernel
 * actually applied it (CONFIG_SCHED_DEADLINE present), 0 on no-op. */
extern int nros_zephyr_set_current_deadline(unsigned int deadline_us);
/* printk — loud channel for placement/deadline application notes. Zephyr's
 * printk returns void (sys/printk.h); signature must match exactly. */
extern void printk(const char* fmt, ...);

/* phase-296 W5.5/W5.7 (C/C++ arm) — apply the tier's kernel EDF deadline on
 * the CALLING thread when the tier is real-time and carries a deadline.
 * Mirrors the Rust arm's `apply_tier_deadline`: the marker line prints ONLY
 * when the shim reports the kernel actually applied it (else a
 * CONFIG_SCHED_DEADLINE-less image would claim EDF while nothing happened).
 * The literal matches `nros_tests::output::ZEPHYR_EDF_DEADLINE_MARKER`. */
static void zephyr_apply_tier_deadline(const char* name, const char* tier_class,
                                       uint64_t deadline_us) {
    if (tier_class == NULL || strcmp(tier_class, "real_time") != 0 || deadline_us == 0u) {
        return;
    }
    unsigned int us = (deadline_us > 0xFFFFFFFFull) ? 0xFFFFFFFFu : (unsigned int)deadline_us;
    if (nros_zephyr_set_current_deadline(us) != 0) {
        printk("nros: EDF deadline set tier=`%s` %uus\n", (name != NULL) ? name : "?", us);
    }
}

/* issue 0655 — the placement dim's accept/fallback marker, for a tier whose
 * pin was applied by the shim in its create->start window. ONE spelling shared
 * by the spawn path here and the Rust arm's `report_core_pin`; the literal
 * prefixes MIRROR nros_tests::output::ZEPHYR_CORE_PIN_MARKER and
 * ZEPHYR_CORE_PIN_FALLBACK_MARKER — keep all three in lockstep. */
static void zephyr_report_core_pin(const char* name, uint32_t core_plus1, int rc) {
    int cpu = (int)(core_plus1 - 1u);
    if (rc == 0) {
        printk("nros: core pin tier=`%s` cpu=%d\n", name, cpu);
    } else {
        printk("nros: core pin FAILED tier=`%s` cpu=%d rc=%d "
               "(-22/EINVAL: the thread was already RUNNING — Zephyr accepts a cpu mask only "
               "before start, issue 0655; -88/ENOSYS: image lacks CONFIG_SCHED_CPU_MASK) "
               "— tier runs unpinned\n",
               name, cpu, rc);
    }
}

/* The BOOT tier's placement attempt (issue 0655).
 *
 * phase-296 W5.7 added this as a general "pin the calling thread" consumer for
 * both the boot tier and every spawned tier. It could never work: Zephyr's
 * `cpu_mask_mod` accepts a mask only on a thread prevented from running, and
 * `k_current_get()` never is. Spawned tiers now pin in their create->start
 * window; this remains only for the boot tier, which the kernel started before
 * run_tiers saw it and which therefore has no such window. A `core` declared on
 * the boot tier CANNOT be honored on Zephyr — reported as the limitation it is,
 * per RFC-0052's fail-loud contract. Mirrors the Rust arm's
 * `apply_boot_tier_core_pin`. */
static void zephyr_apply_boot_core_pin(const char* name, uint32_t core_plus1) {
    if (core_plus1 == 0u) {
        return; /* unpinned */
    }
    int cpu = (int)(core_plus1 - 1u);
    int rc = nros_zephyr_thread_cpu_pin(cpu);
    if (rc == 0) {
        printk("nros: core pin tier=`%s` cpu=%d\n", (name != NULL) ? name : "?", cpu);
    } else {
        printk("nros: core pin FAILED tier=`%s` cpu=%d rc=%d — this is the BOOT tier, which "
               "Zephyr started before nros ran, and a cpu mask is only settable before a thread "
               "starts (issue 0655). Declare the `core` on a SPAWNED tier. -88/ENOSYS instead "
               "means the image lacks CONFIG_SCHED_CPU_MASK. Tier runs unpinned\n",
               (name != NULL) ? name : "?", cpu, rc);
    }
}

/* --- nros-cpp C FFI forward declarations ---
 *
 * These symbols are defined in nros-cpp (Rust) and linked into the final binary.
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
 * override on boards that must block for DHCP / link-up. On the canonical
 * Zephyr path (CONFIG_NET_CONFIG_AUTO_INIT) this is a no-op — the kernel
 * brings up networking before main() runs. */
extern void nros_board_network_wait(void);

/* RFC-0034 — the sole sanctioned allocation seam (wraps the Zephyr heap); a
 * single counter sees all traffic. Direct k_malloc/k_free are forbidden. */
extern void* nros_platform_alloc(size_t size);
extern void nros_platform_dealloc(void* ptr);

/* --- Executor storage sizing ---
 *
 * nros_cpp_init / nros_cpp_executor_open_over_session both need storage of
 * NROS_CPP_EXECUTOR_STORAGE_SIZE bytes (cmake-generated
 * nros_cpp_config_generated.h), 8-byte aligned. In the Zephyr module build
 * that header IS on the include path (nros-rust/nros-cpp-generated), so use
 * the REAL per-build value. issue #245: the old hardcoded "NuttX fallback
 * (79304) rounded up to 80 KiB" (81920) silently became 32 bytes SHORT when
 * the executor grew to 81952 — the tier executor's tail then overwrote the
 * next Zephyr sys_heap chunk header, corrupting the system heap the first
 * time subscriber-delivery state touched the tail (remote-subscriber-gated
 * crash in z_declare_publisher's free path). Never hardcode this again; the
 * fallback exists ONLY for builds where the generated header is absent, and
 * carries generous headroom.
 * nros_platform_alloc on the Zephyr heap returns 8-byte aligned memory. */
#if defined(__has_include)
#if __has_include(<nros/nros_cpp_config_generated.h>)
#include <nros/nros_cpp_config_generated.h>
#endif
#endif
#ifdef NROS_CPP_EXECUTOR_STORAGE_SIZE
#define NROS_ZEPHYR_EXECUTOR_STORAGE_BYTES ((NROS_CPP_EXECUTOR_STORAGE_SIZE + 7u) & ~7u)
#else
#define NROS_ZEPHYR_EXECUTOR_STORAGE_BYTES 98304u /* 96 KiB fallback headroom */
#endif

/* --- Local tier-spec type ---
 *
 * Mirror of nros_native_tier_spec_t (nros/main.h). Layout MUST match: same
 * field order, same types, same ABI (verified by the C++ caller casting
 * NativeTierSpec* → nros_native_tier_spec_t* → this type). */
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
    /* phase-296 W5.7 — appended: the RTOS-agnostic real-time policy
     * (NULL/0 = unset). This mirror consumes tier_class + deadline_us
     * (Zephyr kernel EDF); the rest ride for layout parity. */
    const char* tier_class;
    uint64_t period_us;
    uint64_t budget_us;
    uint64_t deadline_us;
    const char* deadline_policy;
} nros_tier_spec_t;

/* --- Per-tier task context ---
 *
 * Heap-allocated by the spawning thread before nros_zephyr_tier_task_create;
 * lives for the firmware lifetime (the spawned task never returns).
 * executor_storage is a separate heap block passed to
 * nros_cpp_executor_open_over_session. */
typedef struct {
    void* session_handle;
    uint32_t domain_id;
    const char* const* groups;
    size_t n_groups;
    uint64_t spin_period_us;
    nros_tier_setup_fn_t setup;
    void* executor_storage;
    /* issue #144 — chained spawn tail: the tiers still to bring up AFTER this
     * one. This task spawns rest[0] (carrying rest[1..]) only after its own
     * setup returns, so no two setups overlap on the shared session. */
    const nros_tier_spec_t* rest;
    size_t n_rest;
    /* phase-296 W5.5/W5.7 — tier name + declared CPU pin (+1; 0 = unpinned)
     * + generic real-time policy, carried so the tier task can self-apply
     * its kernel placement/deadline. */
    const char* name;
    uint32_t core_plus1;
    const char* tier_class;
    uint64_t deadline_us;
} nros_zephyr_tier_ctx_t;

/* Forward decl — zephyr_tier_task and zephyr_spawn_next_tier are mutually
 * recursive (each tier's task spawns the next tier via this helper). */
static int zephyr_spawn_next_tier(void* session_handle, uint8_t domain_id,
                                  const nros_tier_spec_t* remaining, size_t n_remaining);

/* Minimum spin delay: 1 ms. */
#define SPIN_PERIOD_FLOOR_MS 1u

/* zephyr_tier_task — body of each non-boot tier task.
 *
 * Opens a borrowed executor over the shared session, gates it to the tier's
 * callback groups, calls the tier's setup function, spawns the next tier
 * (issue #144), then spins forever at the tier's declared period. On failure:
 * parks (never returns — the boot thread is the session owner and must outlive
 * all borrowed executors). Signature is `void* (*)(void*)` to match the
 * nros_zephyr_tier_task_create shim's entry type. */
static void* zephyr_tier_task(void* arg) {
    nros_zephyr_tier_ctx_t* ctx = (nros_zephyr_tier_ctx_t*)arg;

    /* issue 0655 — NO core pin here. This runs on the tier's own STARTED
     * thread, and Zephyr refuses a cpu mask on a started thread, so the
     * self-apply this line used to do could only ever log -EINVAL. The pin is
     * applied inside nros_zephyr_tier_task_create, before the start. */

    /* issue 0260 — but this IS the place to report where the tier actually
     * runs. The pin markers say the kernel ACCEPTED the mask; on a
     * uniprocessor image that is true and uninformative, since cpu 0 is the
     * only CPU the thread could be on. Mirrors the Rust arm's
     * `nros: core pin observed` in `entry_tiers.rs` — the two must not drift,
     * which is why both were added together rather than one now and one when
     * an SMP fixture needs it.
     *
     * Silent when the image cannot answer, rather than printing a fabricated
     * cpu 0. */
    {
        uint32_t observed = nros_zephyr_current_cpu();
        if (observed != 0xFFFFFFFFu) {
            printk("nros: core pin observed tier=`%s` running_on=%u\n",
                   (ctx->name != NULL) ? ctx->name : "?", observed);
        }
    }

    zephyr_apply_tier_deadline(ctx->name, ctx->tier_class, ctx->deadline_us);

    /* Open a borrowed executor that shares the primary session. The primary
     * executor (boot thread) must outlive this task — the boot spin loop runs
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
            nros_zephyr_msleep(1000);
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
                nros_zephyr_msleep(1000);
            }
        }
    }

    /* issue #144 — this tier's setup is done, so bringing up the next tier can
     * no longer race our declares: spawn rest[0] (carrying rest[1..]). A failed
     * DOWNSTREAM spawn must NOT stop this tier spinning its own work, so ignore
     * the return (zephyr_spawn_next_tier frees what it allocated on failure). */
    (void)zephyr_spawn_next_tier(ctx->session_handle, (uint8_t)ctx->domain_id, ctx->rest,
                                 ctx->n_rest);

    /* Spin loop. Pass the tier period as the spin_once timeout — a BLOCKING
     * read drives the shared session's TX/handshake from the spin path and
     * yields the CPU cooperatively (mirrors the Rust zephyr run_tiers tier
     * loop, which spins with a real period and no extra sleep). */
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
    for (;;) {
        uint64_t iter_start_ns = nros_platform_clock_ns();
        nros_cpp_spin_once(ctx->executor_storage, (int32_t)period_ms);
        gap_state = nros_tier_spin_gap_step(gap_state, iter_start_ns, nros_platform_clock_ns(),
                                            ctx->spin_period_us);
    }

    /* Unreachable — the spin loop never exits; satisfies the non-void return. */
    return NULL;
}

/* zephyr_spawn_next_tier — issue #144 chained tier spawn.
 *
 * Spawns exactly ONE k_thread (via the shim) for remaining[0], handing it
 * remaining[1..] as its own `rest` so the chain continues once its setup
 * completes. Empty `remaining` → nothing left, return 0. Serializing spawns
 * behind each setup guarantees no two setup() (entity declare) calls run
 * concurrently on the shared zenoh-pico session — the interest-handshake race
 * that silently closes a losing publisher's write filter.
 *
 * On any alloc/create failure, frees what IT allocated and returns -1. It does
 * NOT touch the caller's storage. */
static int zephyr_spawn_next_tier(void* session_handle, uint8_t domain_id,
                                  const nros_tier_spec_t* remaining, size_t n_remaining) {
    if (n_remaining == 0u) {
        return 0;
    }
    const nros_tier_spec_t* t = &remaining[0];

    /* Allocate executor storage for this tier. */
    void* tier_exec = nros_platform_alloc(NROS_ZEPHYR_EXECUTOR_STORAGE_BYTES);
    if (tier_exec == NULL) {
        return -1;
    }
    memset(tier_exec, 0, NROS_ZEPHYR_EXECUTOR_STORAGE_BYTES);

    /* Allocate the tier task context (lives for firmware lifetime). */
    nros_zephyr_tier_ctx_t* ctx =
        (nros_zephyr_tier_ctx_t*)nros_platform_alloc(sizeof(nros_zephyr_tier_ctx_t));
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
    /* Chain tail: this task will spawn remaining[1] after its own setup. */
    ctx->rest = remaining + 1;
    ctx->n_rest = n_remaining - 1u;
    ctx->name = t->name;
    ctx->core_plus1 = t->core_plus1;
    ctx->tier_class = t->tier_class;
    ctx->deadline_us = t->deadline_us;

    /* RAW Zephyr priority from the tier spec (the system.toml
     * [tiers.*.zephyr].priority is the numeric Zephyr value verbatim —
     * negatives are cooperative). Clamp to int32 range. The shim manages the
     * tier's stack from its static pool (NROS_ZEPHYR_TIER_STACK_SIZE);
     * t->stack_bytes rides along so the shim can print LOUD when the
     * declared stack exceeds the fixed slot (phase-302 W2, issue 0262). */
    int64_t p = t->priority;
    if (p > (int64_t)INT32_MAX) {
        p = (int64_t)INT32_MAX;
    } else if (p < (int64_t)INT32_MIN) {
        p = (int64_t)INT32_MIN;
    }

    /* issue 0655 — the pin happens INSIDE the shim, between k_thread_create and
     * k_thread_start. An unpinned tier passes core_plus1 = 0 and keeps the old
     * K_NO_WAIT path verbatim. */
    int pin_rc = 0;
    int rc = nros_zephyr_tier_task_create(zephyr_tier_task, ctx, (int32_t)p,
                                          (t->name != NULL) ? t->name : "nros_tier",
                                          (size_t)t->stack_bytes, t->core_plus1, &pin_rc);
    if (rc != 0) {
        nros_platform_dealloc(ctx);
        nros_platform_dealloc(tier_exec);
        return -1;
    }
    if (t->core_plus1 != 0u) {
        zephyr_report_core_pin((t->name != NULL) ? t->name : "?", t->core_plus1, pin_rc);
    }
    return 0;
}

/* nros_board_zephyr_run_tiers — phase-281 W3a (RFC-0015 Model 1 embedded).
 *
 * Called from ZephyrBoard::run_tiers (main.hpp) which is called from the
 * generated `int main(void)`. By this point: the Zephyr kernel is running, the
 * network is up (CONFIG_NET_CONFIG_AUTO_INIT), and we are executing on the
 * main() thread.
 *
 * The `tiers` array is laid out identically to nros_native_tier_spec_t (the
 * caller casts NativeTierSpec* → nros_native_tier_spec_t* → nros_tier_spec_t*).
 *
 * `locator`      — zenoh connect endpoint (compile-time NROS_ENTRY_LOCATOR)
 * `domain_id`    — ROS domain ID (compile-time NROS_ENTRY_DOMAIN_ID)
 * `session_name` — primary session / node name; NULL or empty → "node"
 * `tiers`        — tier-spec array in codegen order (descending RAW priority
 *                  number; Zephyr is lower-number-wins, so tiers[0] — the
 *                  boot tier — is the LOWEST-priority tier; issue 0251)
 * `n_tiers`      — number of tiers (>= 1)
 *
 * Returns: normally never returns (the boot tier spins forever). Returns a
 * non-zero error code only if the primary session open or a spawn fails. */
int32_t nros_board_zephyr_run_tiers(const char* locator, uint8_t domain_id,
                                    const char* session_name, const nros_tier_spec_t* tiers,
                                    size_t n_tiers) {
    if (tiers == NULL || n_tiers == 0) {
        return -3; /* NROS_CPP_RET_INVALID_ARGUMENT */
    }

    /* Weak network-readiness gate (no-op on the canonical Zephyr
     * auto-init path; a board/app may provide a strong override). */
    nros_board_network_wait();

    /* issue 0758 W4 — acquire the wall-clock epoch HERE, in the one window
     * where both halves of the ordering hold: the network is up (the gate
     * above just returned) and no component has constructed yet, so nothing
     * has stamped a message. Acquiring earlier cannot reach a server;
     * acquiring later means the first stamps carry boot-relative time and a
     * validating peer rejects exactly the messages a system emits at startup.
     *
     * Unconditional on purpose — the shim decides. The Rust tier arm
     * (`entry_tiers.rs`) makes the same call at the same point, and it cannot
     * hold a `#ifdef CONFIG_NROS_SNTP_EPOCH` because Kconfig knobs do not reach
     * the cargo lane (issue 0460). One decision in C is the only shape in which
     * the two arms cannot drift. */
    nros_zephyr_epoch_acquire_configured();

    /* phase-8 W4 — install the CTF sink BEFORE the executor is opened, so the
     * registration events emitted as callbacks are added are not lost. A sink
     * installed later is not an error, but every callback registered before it
     * decodes as a bare `handle N` with no name. */
    nros_set_trace_sink(nros_zephyr_trace_marker);

    /* --- Open the primary (owning) executor on the boot thread --- */
    const char* sn = (session_name != NULL && session_name[0] != '\0') ? session_name : "node";

    /* Allocate executor storage from the Zephyr heap (8-byte aligned). */
    void* boot_storage = nros_platform_alloc(NROS_ZEPHYR_EXECUTOR_STORAGE_BYTES);
    if (boot_storage == NULL) {
        return -1; /* NROS_CPP_RET_ERROR */
    }
    memset(boot_storage, 0, NROS_ZEPHYR_EXECUTOR_STORAGE_BYTES);

    int rc = nros_cpp_init(locator, domain_id, sn, NULL, boot_storage);
    if (rc != 0) {
        nros_platform_dealloc(boot_storage);
        return (int32_t)rc;
    }

    /* Retrieve the session handle so non-boot tiers can borrow it. The handle
     * remains valid as long as boot_storage lives (it lives forever — the boot
     * spin loop never returns). */
    void* session_handle = nros_cpp_executor_session_handle(boot_storage);

    /* --- Run boot tier (tiers[0]) setup on THIS thread FIRST --- */
    /* issue #144 — boot setup runs BEFORE any tier spawn: concurrent entity
     * declares from two threads race the zenoh-pico interest handshake, and the
     * losing publisher's write filter stays closed (every put silently
     * dropped). Running boot's declares first, then CHAINING the remaining
     * spawns, makes setup order total (boot, t1, t2, …) so no two declares
     * overlap. */
    const nros_tier_spec_t* boot = &tiers[0];

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

    /* --- Kick off the chained spawn (tiers[1] carrying tiers[2..]) --- */
    /* A boot-side spawn failure is fatal: tear down boot_storage (which the
     * helper never touches) and return error. Downstream tier tasks handle
     * their own spawn failures by parking + continuing to spin. */
    int src = zephyr_spawn_next_tier(session_handle, domain_id, &tiers[1], n_tiers - 1u);
    if (src != 0) {
        nros_cpp_fini(boot_storage);
        nros_platform_dealloc(boot_storage);
        return -1;
    }

    /* The boot thread runs tiers[0] itself — adopt its declared RAW priority
     * (the spawned tiers already got theirs at k_thread_create; without this
     * the boot tier keeps the main-thread default and the declared tier QoS
     * would not hold for it). */
    {
        int64_t bp = boot->priority;
        if (bp > (int64_t)INT32_MAX) {
            bp = (int64_t)INT32_MAX;
        } else if (bp < (int64_t)INT32_MIN) {
            bp = (int64_t)INT32_MIN;
        }
        nros_zephyr_set_current_priority((int32_t)bp);
    }
    /* phase-296 W5.5/W5.7 — boot tier self-applies its declared CPU pin +
     * kernel EDF deadline too. */
    zephyr_apply_boot_core_pin(boot->name, boot->core_plus1);
    zephyr_apply_tier_deadline(boot->name, boot->tier_class, boot->deadline_us);

    /* Boot tier spin loop — runs forever. Blocking-read spin_once (period as
     * timeout) so the boot session's zenoh handshake is driven from the spin
     * path and the CPU yields cooperatively (mirrors the Rust zephyr
     * run_tiers boot loop). */
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
