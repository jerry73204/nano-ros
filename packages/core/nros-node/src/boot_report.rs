//! A fixed RAM record the image writes about itself, for targets where no log
//! sink reaches a human.
//!
//! # Why this exists
//!
//! phase-412 derived six pool counts and the executor arena for the
//! mr-canhubk344 island, and then could not tell whether the derived image was
//! correct. The only available signal was the ROS graph's node count, and one
//! unchanged configuration produced 4, 0, 0, 4, 4 across five runs. Every other
//! channel was already disqualified for that board:
//!
//! * The console is on `lpuart0`, which is not wired on the MR-CANHUBK344.
//!   `lpuart2` is the zenoh serial transport and cannot carry a second protocol.
//! * `nros_log` therefore reaches nothing. Both arena diagnostics (issue 0900)
//!   go through it, so the two messages written specifically to explain an
//!   arena failure are invisible on the one board that needed them.
//! * SEGGER RTT was tried and could not discriminate: a working image and a
//!   derived image both emitted only the Zephyr banner, and a deliberate
//!   positive control produced nothing at all.
//! * Semihosting halts the core until a probe answers and FAULTS with no probe
//!   attached, so an image carrying it cannot run standalone.
//!
//! What all of those share is that they are STREAMS: they need the board to
//! still be running, and they need somebody attached at the moment the
//! interesting thing happens. The failure this campaign is trying to see is the
//! opposite shape. An under-sized arena halts DURING entity creation, before
//! the first spin, so issue 0900's advisory never prints -- the failure cannot
//! report itself through any stream.
//!
//! So this is not a stream. It is a fixed-size record in RAM that the image
//! keeps up to date as it boots, read out AFTERWARDS by halting the core and
//! dumping memory. It survives the halt because it does not depend on anything
//! still running, and a PARTIAL record is the useful case rather than a lost
//! one: the last stage reached and the allocation that did not fit are exactly
//! what names the knob to change.
//!
//! # Reading it
//!
//! The record is a `#[no_mangle]` static, so it has a symbol in the ELF and a
//! debugger can find it without the address being wired in anywhere:
//!
//! ```text
//! pyocd commander -t s32k344 -c "halt" -c "savemem <addr> <len> report.bin"
//! python3 scripts/read-boot-report.py <elf> report.bin
//! ```
//!
//! [`MAGIC`] distinguishes a written record from uninitialised RAM, and
//! [`BootReport::struct_size`] lets a reader refuse a layout it does not know
//! rather than decode it wrongly.
//!
//! # Cost, and why it is opt-in
//!
//! Enabled by setting `NROS_BOOT_REPORT=1` at build time (Zephyr:
//! `CONFIG_NROS_BOOT_REPORT=y`), which makes `nros-node`'s build script emit
//! `cfg(nros_boot_report)`. With the cfg absent every function here is an empty
//! `#[inline(always)]` body and the static does not exist, so an image that
//! does not opt in is byte-identical to one built before this module -- the
//! same rule issue 0900's arena knob and phase-403's `rx_buffer_from_type()`
//! both keep.
//!
//! Enabled, it costs [`BootReport::struct_size`] bytes of `.bss` (60 on a
//! 32-bit target) and a handful of relaxed atomic stores on paths that run once
//! per entity at registration. Nothing here is on the spin path.

#![allow(clippy::module_name_repetitions)]

/// `"NRSR"` -- nano-ros self report. Written LAST, so a reader that finds it
/// knows every field before it is already valid.
pub const MAGIC: u32 = 0x4e52_5352;

/// Layout version. Bump on any field change; a reader refuses what it does not
/// know rather than decoding a record it would misread.
pub const VERSION: u32 = 2;

/// How far boot got. Monotonic, and the single most useful field: an arena
/// failure halts during entity creation, so the stage that was NOT reached
/// names the phase to look at.
///
/// Numbers follow EXECUTION ORDER, because `checkpoint` keeps the maximum
/// and a stage that runs earlier but numbers higher would make the record
/// claim less progress than was made. Inserting one therefore renumbers the
/// rest and bumps [`VERSION`]; the decoder refuses a version it does not
/// know rather than misreading it, which is what makes that safe.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum Stage {
    /// RAM as the loader left it. Never stored; a reader seeing this with a
    /// valid magic has found a record that was reset but not re-entered.
    Untouched = 0,
    /// The record itself is initialised and the compile-time knobs are in it.
    ///
    /// Stamped at the TOP of the C++ entry point, before any argument is
    /// validated, so "the image never entered nano-ros" is distinguishable
    /// from "it entered and died before the executor". Version 1 stamped this
    /// inside the executor constructor instead, which made those two cases
    /// identical -- both read magic 0 -- and cost a disassembly walk to tell
    /// apart on the first board run.
    ReportReady = 1,
    /// The boot config resolved: node name, namespace, locator and domain id
    /// all parsed. Everything between here and [`Stage::ReportReady`] is
    /// argument validation, and [`BootReport::cpp_init_ret`] says which check
    /// rejected it.
    BootConfigResolved = 2,
    /// An `Executor` has bound its arena, so [`BootReport::arena_capacity`]
    /// is the real slice length rather than the compiled constant.
    ExecutorReady = 3,
    /// Entity registration has begun. The interval between this and
    /// [`Stage::EntitiesReady`] is where an under-sized arena halts.
    RegisteringEntities = 4,
    /// Every entity the image declares was registered successfully.
    EntitiesReady = 5,
    /// The first `spin_once` was entered, which is where issue 0900's
    /// headroom advisory would have printed had a sink existed.
    FirstSpin = 6,
}

#[cfg(nros_boot_report)]
pub use enabled::*;

#[cfg(nros_boot_report)]
mod enabled {
    use super::{MAGIC, Stage, VERSION};
    use portable_atomic::{AtomicU32, Ordering};

    /// The record. One per image, in `.bss`.
    ///
    /// `#[repr(C)]` with every field an `AtomicU32` -- which is
    /// `repr(transparent)` over `u32` -- so the layout is exactly the sequence
    /// of 32-bit words the reader script decodes, on every target this crate
    /// builds for.
    ///
    /// Atomics rather than a `static mut` because the record is written from
    /// registration paths that an application may reach from more than one
    /// thread. `Relaxed` throughout: there is no ordering relationship to
    /// establish with any other data, and the reader is a debugger that has
    /// already halted the core.
    #[repr(C)]
    pub struct BootReport {
        magic: AtomicU32,
        version: AtomicU32,
        struct_size: AtomicU32,
        stage: AtomicU32,

        // Compile-time, so that comparing these against what the build system
        // BELIEVES it delivered turns a "derived value did not arrive" defect
        // into a measurement. `scripts/check-knob-delivery.py` asserts the same
        // identity one step earlier, at `build.ninja`; this is the same
        // assertion made by the silicon.
        arena_size: AtomicU32,
        max_cbs: AtomicU32,
        max_sc: AtomicU32,
        max_nodes: AtomicU32,
        default_rx_buf_size: AtomicU32,

        // Runtime.
        arena_capacity: AtomicU32,
        arena_used: AtomicU32,
        alloc_count: AtomicU32,
        last_alloc_size: AtomicU32,
        /// Bytes the allocation that FAILED asked for, or 0 if none has.
        failed_alloc_size: AtomicU32,
        /// Bytes by which that allocation overran the arena. This is the
        /// number to add to `NROS_EXECUTOR_ARENA_SIZE`, which is why it is
        /// stored rather than left to be recomputed from the two above.
        failed_alloc_shortfall: AtomicU32,
        /// `nros_cpp_init`'s return code, as the two's-complement bits of an
        /// `i32`, or 0 (`NROS_CPP_RET_OK`) if it has not returned yet.
        ///
        /// The stage says HOW FAR init got; this says why it stopped. Without
        /// it, every early return in that function -- a null argument, a
        /// non-UTF-8 name, a bad domain id, a backend that refused to open --
        /// is one indistinguishable "did not reach the executor".
        cpp_init_ret: AtomicU32,
    }

    impl BootReport {
        const fn new() -> Self {
            Self {
                magic: AtomicU32::new(0),
                version: AtomicU32::new(0),
                struct_size: AtomicU32::new(0),
                stage: AtomicU32::new(0),
                arena_size: AtomicU32::new(0),
                max_cbs: AtomicU32::new(0),
                max_sc: AtomicU32::new(0),
                max_nodes: AtomicU32::new(0),
                default_rx_buf_size: AtomicU32::new(0),
                arena_capacity: AtomicU32::new(0),
                arena_used: AtomicU32::new(0),
                alloc_count: AtomicU32::new(0),
                last_alloc_size: AtomicU32::new(0),
                failed_alloc_size: AtomicU32::new(0),
                failed_alloc_shortfall: AtomicU32::new(0),
                cpp_init_ret: AtomicU32::new(0),
            }
        }

        /// Size of the record in bytes, as the reader must expect it.
        ///
        /// ASKED OF THE COMPILER, not counted by hand. A hand-written word
        /// count is a second statement of the field list that drifts the first
        /// time a field is added, and it would drift SILENTLY -- the reader
        /// would accept the record and decode one field short. This whole
        /// campaign is about not hand-picking numbers the build already knows.
        #[must_use]
        pub const fn struct_size() -> u32 {
            // The record is all `AtomicU32`, so this is exact on every target
            // and there is no padding for the cast to lose.
            core::mem::size_of::<Self>() as u32
        }
    }

    /// A plain-value copy of the record.
    ///
    /// Field-for-field with [`BootReport`] and in the SAME ORDER, because
    /// `scripts/read-boot-report.py` decodes that order out of a memory dump.
    /// A test that reads through this therefore exercises the same layout the
    /// script assumes, which is the only thing keeping the two in step.
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub struct Snapshot {
        pub magic: u32,
        pub version: u32,
        pub struct_size: u32,
        pub stage: u32,
        pub arena_size: u32,
        pub max_cbs: u32,
        pub max_sc: u32,
        pub max_nodes: u32,
        pub default_rx_buf_size: u32,
        pub arena_capacity: u32,
        pub arena_used: u32,
        pub alloc_count: u32,
        pub last_alloc_size: u32,
        pub failed_alloc_size: u32,
        pub failed_alloc_shortfall: u32,
        pub cpp_init_ret: u32,
    }

    /// Read the record.
    #[must_use]
    pub fn snapshot() -> Snapshot {
        let r = &NROS_BOOT_REPORT;
        let g = |f: &AtomicU32| f.load(Ordering::Relaxed);
        Snapshot {
            magic: g(&r.magic),
            version: g(&r.version),
            struct_size: g(&r.struct_size),
            stage: g(&r.stage),
            arena_size: g(&r.arena_size),
            max_cbs: g(&r.max_cbs),
            max_sc: g(&r.max_sc),
            max_nodes: g(&r.max_nodes),
            default_rx_buf_size: g(&r.default_rx_buf_size),
            arena_capacity: g(&r.arena_capacity),
            arena_used: g(&r.arena_used),
            alloc_count: g(&r.alloc_count),
            last_alloc_size: g(&r.last_alloc_size),
            failed_alloc_size: g(&r.failed_alloc_size),
            failed_alloc_shortfall: g(&r.failed_alloc_shortfall),
            cpp_init_ret: g(&r.cpp_init_ret),
        }
    }

    /// The record, findable by symbol name from a debugger.
    ///
    /// `#[used]` because nothing in a minimal image necessarily reads it, and a
    /// static whose only writes are through this module's functions is exactly
    /// what a linker is entitled to discard.
    #[unsafe(no_mangle)]
    #[used]
    pub static NROS_BOOT_REPORT: BootReport = BootReport::new();

    /// Stamp the header and the compile-time knobs.
    ///
    /// Idempotent, and safe to call from more than one place -- an image with
    /// two executors should not have to decide which one owns the record.
    /// MAGIC is stored LAST so a reader that finds it knows the rest is there.
    pub fn init() {
        let r = &NROS_BOOT_REPORT;
        r.version.store(VERSION, Ordering::Relaxed);
        r.struct_size
            .store(BootReport::struct_size(), Ordering::Relaxed);
        r.arena_size
            .store(saturate(crate::config::ARENA_SIZE), Ordering::Relaxed);
        r.max_cbs
            .store(saturate(crate::config::MAX_CBS), Ordering::Relaxed);
        r.max_sc
            .store(saturate(crate::config::MAX_SC), Ordering::Relaxed);
        r.max_nodes
            .store(saturate(crate::config::MAX_NODES), Ordering::Relaxed);
        r.default_rx_buf_size.store(
            saturate(crate::config::DEFAULT_RX_BUF_SIZE),
            Ordering::Relaxed,
        );
        r.magic.store(MAGIC, Ordering::Relaxed);
        checkpoint(Stage::ReportReady);
    }

    /// Record that boot reached `stage`.
    ///
    /// MONOTONIC: a lower stage never overwrites a higher one, so a late call
    /// on a re-entered path cannot make the record claim less progress than was
    /// actually made. That matters because the field's whole purpose is to be
    /// believed about a boot that did not finish.
    pub fn checkpoint(stage: Stage) {
        let want = stage as u32;
        let r = &NROS_BOOT_REPORT;
        let mut cur = r.stage.load(Ordering::Relaxed);
        while want > cur {
            match r
                .stage
                .compare_exchange_weak(cur, want, Ordering::Relaxed, Ordering::Relaxed)
            {
                Ok(_) => return,
                Err(actual) => cur = actual,
            }
        }
    }

    /// Record the arena slice an `Executor` actually bound.
    ///
    /// Not the same number as `ARENA_SIZE`, and the difference is a finding
    /// rather than noise: the arena's placement is the caller's choice
    /// (issue 0900), so an image can compile one size and hand the executor
    /// another. Both are in the record so a dump can say which happened.
    pub fn note_arena_capacity(capacity: usize) {
        NROS_BOOT_REPORT
            .arena_capacity
            .store(saturate(capacity), Ordering::Relaxed);
    }

    /// Record a successful arena allocation.
    pub fn note_alloc(size: usize, used_after: usize) {
        let r = &NROS_BOOT_REPORT;
        r.alloc_count.fetch_add(1, Ordering::Relaxed);
        r.last_alloc_size.store(saturate(size), Ordering::Relaxed);
        r.arena_used.store(saturate(used_after), Ordering::Relaxed);
    }

    /// Record the arena allocation that did not fit.
    ///
    /// FIRST writer wins, on the same reasoning as [`checkpoint`]: the first
    /// failure is the one that explains the boot, and any later one is a
    /// consequence of it.
    pub fn note_alloc_failed(size: usize, shortfall: usize) {
        let r = &NROS_BOOT_REPORT;
        if r.failed_alloc_size
            .compare_exchange(0, saturate(size), Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            r.failed_alloc_shortfall
                .store(saturate(shortfall), Ordering::Relaxed);
        }
    }

    /// Record `nros_cpp_init`'s return code.
    ///
    /// LAST writer wins, unlike [`note_alloc_failed`]: an image may call
    /// `nros_cpp_init` more than once (per component, per tier), and the
    /// interesting one is the call that did not get through, which is the one
    /// that leaves the stage where it stopped.
    pub fn note_cpp_init_ret(ret: i32) {
        NROS_BOOT_REPORT
            .cpp_init_ret
            .store(ret as u32, Ordering::Relaxed);
    }

    /// `usize` -> `u32`, saturating.
    ///
    /// Every field is a `u32` so the record's layout does not change between a
    /// 32-bit board and a 64-bit host running the same tests. Saturating rather
    /// than truncating because a value too large to represent should read as
    /// "enormous", not as its low half -- a truncated 4 GiB reads as 0, which
    /// is the one wrong answer that looks like a normal one.
    fn saturate(v: usize) -> u32 {
        u32::try_from(v).unwrap_or(u32::MAX)
    }
}

#[cfg(not(nros_boot_report))]
pub use disabled::*;

/// No-op stubs, so call sites need no `cfg` of their own and an image that does
/// not opt in is byte-identical.
#[cfg(not(nros_boot_report))]
mod disabled {
    use super::Stage;

    #[inline(always)]
    pub fn init() {}

    #[inline(always)]
    pub fn checkpoint(_stage: Stage) {}

    #[inline(always)]
    pub fn note_arena_capacity(_capacity: usize) {}

    #[inline(always)]
    pub fn note_alloc(_size: usize, _used_after: usize) {}

    #[inline(always)]
    pub fn note_alloc_failed(_size: usize, _shortfall: usize) {}

    #[inline(always)]
    pub fn note_cpp_init_ret(_ret: i32) {}
}

#[cfg(all(test, nros_boot_report))]
mod tests {
    use super::*;

    /// The reader decodes sixteen u32s positionally, so the record must be
    /// exactly that and nothing else -- no padding, no reordering.
    ///
    /// `size_of` on the TARGET, which is the half `check-boot-report-layout.py`
    /// cannot see: that gate compares two source files, and this compares the
    /// source against what the compiler actually laid out.
    #[test]
    fn the_record_is_sixteen_packed_u32s() {
        assert_eq!(BootReport::struct_size(), 16 * 4);
        assert_eq!(
            core::mem::size_of::<BootReport>(),
            16 * core::mem::size_of::<u32>(),
            "the record grew padding; the reader decodes positionally"
        );
        assert_eq!(core::mem::align_of::<BootReport>(), 4);
    }

    /// The magic is written LAST, so finding it means the rest is valid.
    ///
    /// The reader leans on this to tell "the image died before it had an
    /// executor" apart from "the dump is at the wrong address", and it can
    /// only do so if the ordering actually holds.
    #[test]
    fn init_stamps_the_header_and_the_compiled_knobs() {
        init();
        let s = snapshot();
        assert_eq!(s.magic, MAGIC);
        assert_eq!(s.version, VERSION);
        assert_eq!(s.struct_size, BootReport::struct_size());
        assert_eq!(s.arena_size, crate::config::ARENA_SIZE as u32);
        assert_eq!(s.max_cbs, crate::config::MAX_CBS as u32);
        assert_eq!(s.max_nodes, crate::config::MAX_NODES as u32);
        assert!(s.stage >= Stage::ReportReady as u32);
    }

    /// A late call on a re-entered path must not make the record claim LESS
    /// progress than was actually made. The field's whole purpose is to be
    /// believed about a boot that did not finish.
    #[test]
    fn a_checkpoint_never_goes_backwards() {
        init();
        checkpoint(Stage::FirstSpin);
        assert_eq!(snapshot().stage, Stage::FirstSpin as u32);
        checkpoint(Stage::ExecutorReady);
        assert_eq!(
            snapshot().stage,
            Stage::FirstSpin as u32,
            "an earlier stage overwrote a later one"
        );
    }

    /// The FIRST failure is the one that explains the boot; a later one is a
    /// consequence of it and must not overwrite the cause.
    #[test]
    fn the_first_alloc_failure_wins() {
        init();
        note_alloc_failed(100, 8);
        note_alloc_failed(999, 512);
        let s = snapshot();
        assert_eq!(s.failed_alloc_size, 100);
        assert_eq!(s.failed_alloc_shortfall, 8);
    }

    /// A value too large for the field must read as enormous, not as its low
    /// half. A truncated 4 GiB reads as 0, which is the one wrong answer that
    /// looks like a normal one.
    #[test]
    fn an_unrepresentable_size_saturates_rather_than_truncating() {
        init();
        note_arena_capacity(usize::MAX);
        assert_eq!(snapshot().arena_capacity, u32::MAX);
    }
}
