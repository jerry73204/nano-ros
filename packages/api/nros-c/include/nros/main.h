/**
 * @file main.h
 * @ingroup grp_node
 * @brief Phase 219.E — `<nros/main.h>` Entry-pkg header (C variant).
 *
 * Symmetric to `<nros/main.hpp>`: the cmake fn `nano_ros_entry(LAUNCH
 * "<bringup>:<file>.launch.xml")` drives per-Entry-pkg codegen via
 * `nros codegen entry --lang c`, then appends the generated TU to the
 * executable target's sources.
 *
 * This header provides:
 *
 *   1. `NROS_MAIN_C(<board_id>, "<bringup>:<file>.launch.xml")` —
 *      empty-expansion macro the user's TU may carry as a doc / IDE
 *      hint. Declarative only; cmake fn drives codegen.
 *
 *   2. `nros_board_native_run(nros_node_register_fn entry)` — the
 *      C-FFI Board adapter the generated TU calls. Owns the
 *      `nros::init() → entry(context) → nros::spin() →
 *      nros::shutdown()` lifecycle.
 *
 * Phase 212.L.2 keeps Entry pkgs `native`-only at the cmake surface
 * for v1.
 */

#ifndef NROS_MAIN_H
#define NROS_MAIN_H

#include "nros/node_pkg.h"
#include "nros/visibility.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Phase 257 (W0-A, RFC-0043) — typed C Entry lifecycle. The C-ABI sibling of
 * the C++ `LinuxBoard::run_components`; the generated typed C TU (emitted by
 * `nros codegen entry --lang c --typed`) calls this from `main`. `setup` is
 * invoked once after `init`, with the executor handle, to create each node and
 * `configure` its component on the real executor; then this pumps the executor
 * (init → setup → spin → shutdown). Returns 0 on graceful exit, else the first
 * non-zero `setup` / spin code. Defined in nros-cpp (the typed runtime). */
typedef int32_t (*nros_c_entry_setup_fn)(void* executor);
NROS_PUBLIC int32_t nros_board_native_run_components(nros_c_entry_setup_fn setup);

/* Phase 266 (W5b) — named variant: `session_name` sets the primary session /
 * node name visible via `ros2 node list` (the #98 fix for C entries). NULL or
 * empty → falls back to `"node"`. The generated typed C entry (emitted by
 * `nros codegen entry --lang c --typed`) calls this from `main`, passing
 * `nros_boot_config_node_name(&NROS_BOOT_CONFIG)`. Defined in nros-cpp. */
NROS_PUBLIC int32_t nros_board_native_run_components_named(const char* session_name,
                                                           nros_c_entry_setup_fn setup);

/* Phase 274.W2 (RFC-0015 Model 1) — per-tier spec for
 * `nros_board_native_run_tiers`.
 *
 * `name`           — tier name (null-terminated), informational.
 * `groups`         — array of `n_groups` null-terminated callback-group names;
 *                    NULL / 0 means wildcard (accept all groups).
 * `n_groups`       — number of entries in `groups`.
 * `priority`       — raw POSIX nice level (advisory).
 * `stack_bytes`    — informational on native (std::thread manages the stack).
 * `spin_period_us` — sleep between spin_once calls; 0 uses a 1 ms floor.
 * `setup`          — called once on the tier thread (after set_active_groups)
 *                    with the tier executor handle; returns 0 on success or
 *                    NULL to skip setup (tier receives no nodes of its own).
 * `core_plus1`     — RFC-0052 W2: CPU pin + 1; 0 = unpinned. Consumed where
 *                    the kernel offers affinity (SMP builds); ignored with a
 *                    note on uniprocessor targets.
 * `preempt_threshold` — ThreadX preemption threshold; -1 = unset. Bake-time
 *                    validated ThreadX-only (other targets never see it).
 * `tier_class`     — phase-296 W5.7 append: RTOS-agnostic scheduling class
 *                    ("best_effort" | "real_time" | "time_triggered");
 *                    NULL = unset. Static string from codegen.
 * `period_us` / `budget_us` / `deadline_us` — generic real-time policy
 *                    (sporadic period/budget, relative deadline); 0 = unset.
 * `deadline_policy` — on-miss action ("ignore"|"warn"|"skip"|"fault");
 *                    NULL = unset.
 *                    Kernel-native consumers read these where the RTOS offers
 *                    the feature (Zephyr EDF: tier_class=="real_time" +
 *                    deadline_us → k_thread_deadline_set); the cooperative
 *                    executor lowering stays codegen-emitted per tier. */
typedef struct {
    const char* name;
    const char* const* groups;
    size_t n_groups;
    int64_t priority;
    size_t stack_bytes;
    uint64_t spin_period_us;
    nros_c_entry_setup_fn setup;
    uint32_t core_plus1;
    int64_t preempt_threshold;
    /* phase-296 W5.7 — appended. THIS DECLARATION IS CANONICAL. The struct is
     * mirrored by hand in five more places — main.hpp `NativeTierSpec`,
     * nros-cpp `NativeTierSpecC`, and the THREE board `nros_tier_spec_t`
     * mirrors (zephyr / nuttx-qemu / freertos) — and initialised by the two
     * entry templates. ABI append-only: a field inserted anywhere but the end
     * shifts every field after it in whichever mirror missed the edit.
     * Gated by `check-ffi-struct-mirrors` (push lane), which compares all
     * eight sites against this one; the templates use DESIGNATED
     * initialisers so a rename is a compile error at the generated TU too
     * (phase-432, RFC-0091 §5). */
    const char* tier_class;
    uint64_t period_us;
    uint64_t budget_us;
    uint64_t deadline_us;
    const char* deadline_policy;
} nros_native_tier_spec_t;

/* Phase 274.W2 (RFC-0015 Model 1) — run a multi-tier native entry over one
 * shared RMW session. Opens ONE session on the boot thread; spawns one
 * std::thread per non-boot tier, each with a borrowed executor (no second
 * RMW session, no double-close). Each tier: open borrowed executor →
 * set_active_groups → setup(executor) → spin at spin_period_us. Boot thread
 * runs tier[0] on the owning executor and respects $NROS_ENTRY_SPIN_MS.
 * Returns after boot spin exits (NROS_ENTRY_SPIN_MS or spin error) after
 * joining all tier threads and closing the session. Defined in nros-cpp. */
NROS_PUBLIC int32_t nros_board_native_run_tiers(const char* session_name,
                                                const nros_native_tier_spec_t* tiers,
                                                size_t n_tiers);

/* Phase 274.W3 (RFC-0015 Model 1) — run a multi-tier embedded C/C++ entry on
 * FreeRTOS: open ONE RMW session, spawn one FreeRTOS task per non-boot tier
 * (each with a borrowed executor sharing the session), run the boot tier on the
 * caller's task (the startup.c app task). `locator` is the connect endpoint;
 * `domain_id` is the ROS domain id; `session_name` names the primary session.
 * Defined in nros-board-freertos (compiled by board's build.rs glue). */
NROS_PUBLIC int32_t nros_board_freertos_run_tiers(const char* locator, uint8_t domain_id,
                                                  const char* session_name,
                                                  const nros_native_tier_spec_t* tiers,
                                                  size_t n_tiers);

/* phase-281 W3a (RFC-0015 Model 1) — run a multi-tier embedded C/C++ entry on
 * Zephyr: open ONE RMW session on the caller's thread (the Zephyr `main()`
 * thread), spawn one `k_thread` per non-boot tier via the
 * `nros_zephyr_tier_task_create` shim (each with a borrowed executor sharing
 * the session), and run the boot tier on the caller. Zephyr owns boot +
 * networking (CONFIG_NET_CONFIG_AUTO_INIT), so there is no lwIP bring-up.
 * `locator` is the connect endpoint; `domain_id` is the ROS domain id;
 * `session_name` names the primary session. Defined in nros-board-zephyr
 * (compiled into the Zephyr `app` library by zephyr/CMakeLists.txt). */
NROS_PUBLIC int32_t nros_board_zephyr_run_tiers(const char* locator, uint8_t domain_id,
                                                const char* session_name,
                                                const nros_native_tier_spec_t* tiers,
                                                size_t n_tiers);

/* phase-281 W3 (nuttx) (RFC-0015 Model 1) — run a multi-tier embedded C/C++
 * entry on NuttX: open ONE RMW session on the caller's thread (the NuttX
 * `app_main` thread), spawn one `pthread` per non-boot tier (NuttX is POSIX —
 * SCHED_FIFO at the tier's raw priority, per-tier stack; each with a borrowed
 * executor sharing the session), and run the boot tier on the caller. NuttX
 * owns boot + networking (the board FFI main brings up eth0 before app_main,
 * phase-280), so there is no network bring-up. `locator` is the connect
 * endpoint; `domain_id` is the ROS domain id; `session_name` names the primary
 * session. Defined in nros-board-nuttx-qemu (compiled by the board's
 * build.rs via `nuttx_platform_build::compile_run_tiers_seam`). */
NROS_PUBLIC int32_t nros_board_nuttx_run_tiers(const char* locator, uint8_t domain_id,
                                               const char* session_name,
                                               const nros_native_tier_spec_t* tiers,
                                               size_t n_tiers);

#ifdef __cplusplus
} /* extern "C" */
#endif

/* Phase 219.E — `NROS_MAIN_C(<board_id>, "<launch_spec>")` declarative
 * marker. Expands to a sentinel TU-local symbol; the cmake fn detects
 * presence via `target_compile_definitions` to avoid double-emit when
 * the user wrote it. The generated TU (emitted by
 * `nano_ros_entry(LAUNCH …)`) carries the canonical `int main()` body
 * either way.
 *
 * Usage:
 *
 *   #include <nros/main.h>
 *   NROS_MAIN_C(nros_board_native, "demo_bringup:system.launch.xml")
 */
#define NROS_MAIN_C(BoardId, LaunchSpec)                                                           \
    NROS_PUBLIC const unsigned char __nros_entry_macro_present = 1;                                \
    _Static_assert(sizeof(LaunchSpec) > 1, "NROS_MAIN_C: launch spec must be a non-empty literal")

#endif /* NROS_MAIN_H */
