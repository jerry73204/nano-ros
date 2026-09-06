//! The FreeRTOS task-scheduling defaults, in one place.
//!
//! phase-337 W5.d — `nros_board_freertos::Config::default()`, the MPS2 board's
//! `build.rs::emit_nros_app_config` and `cmake/templates/freertos_app_config.c.in`
//! each carried their own copy of these eight numbers, and they had **already
//! drifted**: `app_stack_bytes` read 393216 in Rust, 262144 in the board's
//! `build.rs` C-string mirror and 524288 in the CMake template. That is exactly
//! the silent-drift class this phase exists to remove, so the numbers live here
//! and the emitters read them.
//!
//! This module is `no_std` and dependency-free so the runtime `Config` and the
//! `build.rs` emitter can share it — the emitter itself is in
//! [`crate::freertos_build`], behind `build-helpers`.

/// FreeRTOS's `configMAX_PRIORITIES` in the shared `FreeRTOSConfig.h`. Usable
/// task priorities are `0..=FREERTOS_MAX_PRIORITY - 1`; `xTaskCreate` asserts
/// on anything higher.
pub const FREERTOS_MAX_PRIORITY: u32 = 8;

/// THE normalized-0–31 → raw-FreeRTOS priority conversion (issue 0623).
///
/// This is the one place the mapping exists. It used to exist in two, and they
/// did not agree — which is the same silent-drift class as the numbers above,
/// one level up in the abstraction:
///
/// | path | normalized 16 became |
/// | --- | --- |
/// | `Config::to_freertos_priority` (Rust entry) | **4** — proportional |
/// | `clamp_prio` (C entry, `freertos_c_entry.c`) | **7** — saturating |
///
/// So one config produced two different schedules depending on which entry the
/// image used. Worse on the C side: every default was ≥ 8 (`app_priority` 12,
/// zenoh read/lease and poll 16), so all four SATURATED to 7 and the intended
/// ordering — app below transport — collapsed into "everything equal".
///
/// The conversion is proportional rather than saturating because the scale is a
/// *band*, not a range to be clipped: `31` means "most urgent available" on
/// whatever port, and clipping maps most of the band onto one value.
pub const fn to_freertos_priority(normalized: u8) -> u32 {
    let n = if normalized > 31 { 31 } else { normalized };
    // Round-to-nearest over the 0-31 → 0-7 span: (n * 7 / 31), doubled and
    // offset so integer division rounds instead of truncating.
    (n as u32 * (FREERTOS_MAX_PRIORITY - 1) * 2 + 31) / 62
}

/// Priorities are **RAW FreeRTOS** — `0..configMAX_PRIORITIES-1`, higher = more
/// urgent — the same units a `[tiers.<name>.freertos] priority` is written in
/// (issue 0623).
///
/// They were on a normalized 0–31 scale, and that was the defect: a tier and a
/// transport task both end up at `xTaskCreate` in ONE priority space, so an
/// author comparing `priority = 5` against a transport band that read "16"
/// concluded the tier was below it when it was above. Two numbers that are
/// compared must be in one unit.
///
/// The normalized band still exists as an INPUT spelling — see
/// [`to_freertos_priority`] and the `[node.rt]` arm of the board's TOML parser
/// — so configs written against the old scale keep working. What no longer
/// exists is a normalized value living in this struct, where the thing reading
/// it cannot tell which scale it is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FreertosScheduling {
    /// Application task priority (RAW FreeRTOS).
    pub app_priority: u8,
    /// Application task stack size in bytes.
    pub app_stack_bytes: u32,
    /// zenoh-pico read task priority (RAW FreeRTOS).
    pub zenoh_read_priority: u8,
    /// zenoh-pico read task stack size in bytes.
    pub zenoh_read_stack_bytes: u32,
    /// zenoh-pico lease task priority (RAW FreeRTOS).
    pub zenoh_lease_priority: u8,
    /// zenoh-pico lease task stack size in bytes.
    pub zenoh_lease_stack_bytes: u32,
    /// Network poll task priority (RAW FreeRTOS).
    pub poll_priority: u8,
    /// Network poll interval in milliseconds.
    pub poll_interval_ms: u32,
}

/// The default application task stack, in bytes, when the build sets no
/// override. See [`app_stack_bytes`] for the measurement it comes from.
pub const DEFAULT_APP_STACK_BYTES: u32 = 131072;

impl Default for FreertosScheduling {
    fn default() -> Self {
        Self {
            // Issue 0623 — RAW FreeRTOS now, and these are EXACTLY what the
            // normalized values resolved to, so the schedule is unchanged:
            // `to_freertos_priority(12) == 3`, `(16) == 4`. The historical
            // chain was APP_TASK_PRIORITY=3 -> normalized 12 -> back to 3, and
            // POLL_TASK_PRIORITY=4 -> 16 -> back to 4; the round trip is gone
            // and the constants are the ones that reach `xTaskCreate`.
            //
            // CLAUDE.md's FreeRTOS pitfall entry requires the poll task at
            // >= 4, which is now literally what is written here rather than
            // something a reader has to compute.
            app_priority: 3,
            app_stack_bytes: DEFAULT_APP_STACK_BYTES,
            zenoh_read_priority: 4,
            zenoh_read_stack_bytes: 5120,
            zenoh_lease_priority: 4,
            zenoh_lease_stack_bytes: 5120,
            poll_priority: 4,
            poll_interval_ms: 5,
        }
    }
}

/// Resolve the app-task stack size from the `NROS_FREERTOS_APP_STACK_KB` build
/// override, in the one spelling both readers share.
///
/// The Rust `Config` passes `option_env!("NROS_FREERTOS_APP_STACK_KB")`, a
/// `build.rs` passes `env::var(..).ok().as_deref()`. Both land here so the
/// parser cannot drift.
///
/// The default is **128 KiB, and it is MEASURED** (issue 1146). Every FreeRTOS
/// task stack here is drawn from the FreeRTOS heap (heap_4, `ucHeap[]` in
/// `.bss`, sized in `nros-board-freertos/build.rs`), and a spawned tier whose
/// `stack_bytes` is 0 gets this number too — so it is charged once per task,
/// not once per image.
///
/// What the measurement is: `uxTaskGetStackHighWaterMark` on the app task at
/// the end of the register pass, which is the deepest point bring-up reaches.
/// Eight images on qemu mps2-an385, each run to a live zenoh session:
///
/// | image | after boot bringup | after `Executor::open` | after register |
/// | --- | --- | --- | --- |
/// | rust/talker | 5 040 | 8 952 | 25 160 |
/// | rust/listener | 5 040 | 8 952 | 24 784 |
/// | rust/service-server | 5 040 | 8 952 | 25 936 |
/// | rust/service-client | 5 040 | 8 952 | 26 992 |
/// | rust/action-server | 5 040 | 8 952 | **36 152** |
/// | rust/action-client | 5 040 | 8 952 | 33 584 |
/// | workspaces/rust (2 nodes) | 3 208 | 10 752 | 23 296 |
/// | workspaces/realtime-rust (2 tiers) | 3 208 | — | 22 184 boot / 22 368 tier |
///
/// So the worst in-tree peak is **36 152 bytes** and this default is 3.6x it.
/// Reproduce it from any image's own boot output: `report_stack_peak` in
/// `nros-board-freertos/src/entry.rs` prints the line, so nobody has to patch
/// the board to learn the number again. The table above was taken with a
/// throwaway probe; the shipped line reads 8 bytes HIGHER (36 160 for
/// action-server) because the reporting call has a frame of its own — that
/// delta is the difference between the two, not drift.
///
/// The three claims this replaces were each false, and each had outlived its
/// subject by phases:
///
/// - *"the Rust zenoh executor can exceed 160 KiB opening a FreeRTOS session
///   with lwIP up"* — measured **8 952** bytes at exactly that point, on every
///   one of the eight images. Off by 18x.
/// - *"the phase-212 Entry / run-plan Executor open overflows the older
///   256 KiB (issue #46)"* — whatever issue #46 saw in phase 212, nothing on
///   this path now comes within 7x of 256 KiB.
/// - *"a 10-node macro entry's register pass overflows even 384 KiB, hence the
///   override"* — true, and about an OUT-OF-TREE consumer (the sentinel entry
///   of `a60b80da3`, which ships 896 KiB). It argued for the override, never
///   for the default: an entry that overflows 384 KiB is not served by a
///   384 KiB default either. Nothing in this tree sets the override.
///
/// A bigger entry raises it — `NROS_FREERTOS_APP_STACK_KB`, or a `[node.rt]
/// app_stack_bytes`. Getting it wrong is loud but MISATTRIBUTED, which is worth
/// knowing before you read the log. The bracketing run, same image, same
/// router:
///
/// - `NROS_FREERTOS_APP_STACK_KB=40` → boots, serves goals, and reports
///   `app task stack peak 36160 of 40960 bytes (4800 free)`. The measurement
///   predicts the boundary to within 5 KiB.
/// - `NROS_FREERTOS_APP_STACK_KB=32` → `*** MALLOC FAILED ***` and a hang.
///   **NOT** `*** STACK OVERFLOW: <task> ***`, even though the shared
///   `FreeRTOSConfig.h` sets `configCHECK_FOR_STACK_OVERFLOW 2` and the hook is
///   wired: heap_4 hands out the task stack, so the overflow lands in the
///   ADJACENT heap block header and the next `pvPortMalloc` fails before any
///   context switch can check the pattern. The kernel's own stack check cannot
///   win that race here.
///
/// So the way to know is the boot line, not the failure.
///
/// # Panics
/// On a non-decimal value — a typo'd stack size must not silently become the
/// default and stack-overflow at runtime.
pub const fn app_stack_bytes(kb: Option<&str>) -> u32 {
    match kb {
        Some(s) => parse_kb(s) * 1024,
        None => DEFAULT_APP_STACK_BYTES,
    }
}

/// Const decimal parser for the `NROS_FREERTOS_APP_STACK_KB` build env.
const fn parse_kb(s: &str) -> u32 {
    let bytes = s.as_bytes();
    let mut i = 0;
    let mut acc: u32 = 0;
    while i < bytes.len() {
        let d = bytes[i];
        if !d.is_ascii_digit() {
            panic!("NROS_FREERTOS_APP_STACK_KB must be a decimal integer");
        }
        acc = acc * 10 + (d - b'0') as u32;
        i += 1;
    }
    acc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_stack_override_is_read_in_kib() {
        assert_eq!(app_stack_bytes(Some("256")), 262144);
        assert_eq!(app_stack_bytes(None), DEFAULT_APP_STACK_BYTES);
    }

    #[test]
    fn the_poll_task_outranks_the_app_task() {
        // CLAUDE.md's FreeRTOS pitfall: a poll task that does not outrank the
        // app task starves, and lwIP never drains the LAN9118 RX FIFO.
        let sched = FreertosScheduling::default();
        assert!(sched.poll_priority > sched.app_priority);
    }
}
