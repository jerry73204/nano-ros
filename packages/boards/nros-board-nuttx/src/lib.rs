//! # nros-board-nuttx
//!
//! **Generic NuttX board scaffolding for nano-ros.**
//!
//! Layer-2 entry-point in the board / BSP abstraction described in
//! `docs/design/0012-board-bsp-integration-architecture.md`. Unlike the
//! `nros-board-{freertos, threadx}` siblings, this crate is THIN
//! by design — NuttX owns the kernel build through its own
//! `apps/external/nano-ros/` + `Make.defs` + `Kconfig` integration
//! (see `integrations/nuttx/` and the Phase 152.7 polish). The
//! Cargo side only needs to ship `Config` + `run` + board-init
//! hooks; there is no `build.rs` bundling the NuttX kernel
//! sources here.
//!
//! ## 152.4.A scaffolding
//!
//! Opt-in `reference-qemu` feature pulls the board overlay crate
//! `nros-board-nuttx-qemu` (one crate, both QEMU witnesses) so overlays
//! (`nros-board-px4-fmu-v5-nuttx`, `nros-board-<vendor>-<board>-nuttx`)
//! depend on this crate name + can extend the `Config` shape +
//! patch board-specific init via `#[no_mangle]` hooks.
//!
//! 152.4.B (deferred) carves the per-board `Config` / `init_hardware`
//! variation into a `BoardInit` trait so the per-board crate
//! shrinks to a `pub struct MyBoard; impl BoardInit for MyBoard
//! { ... }`. Today the per-board crate hand-rolls `Config`.
//!
//! ## Public contract
//!
//! Two boot-driver shapes coexist during the 212.N migration:
//!
//! ### Legacy (152.4.B) — config-carrying
//!
//! - `Config` — TOML-loaded network + zenoh config.
//! - `run(Config, FnOnce(&Config) -> Result<(), E>) -> !` — entry
//!   point. For NuttX this is a regular Rust `main` that initialises
//!   nros + drops into the user closure; the NuttX kernel is already
//!   up by the time `main` runs (NuttX init is the OS, not something
//!   this crate boots). Diverges via `std::process::exit`.
//! - `run_generic::<B>(cfg, f) -> !` — kernel-agnostic generic over
//!   the legacy [`nros_board_common::BoardInit`] (which carries a
//!   `type Config`).
//! - `init_hardware()` — board-specific peripheral wakes (sensors,
//!   displays, vendor-specific GPIO that NuttX's `apps/` discovery
//!   doesn't auto-configure).
//!
//! ### Phase 212.N.2 — `BoardEntry`-shaped `run_entry`
//!
//! - [`run_entry`] (free fn) — mirrors the
//!   [`nros_platform::BoardEntry::run`] signature so codegen-emitted
//!   `main.rs` can call it without owning a [`Config`]. Parameterised
//!   on a 212.N.1 [`nros_platform::BoardInit`] impl `B` whose
//!   `init_hardware()` takes no argument (overlay state, if any,
//!   lives in `B`'s impl block or in a separate per-board `Config`
//!   the Entry pkg threads through the `setup` closure).
//! - Returns the [`Result`] the closure produces. NuttX is hosted +
//!   POSIX-shaped: `fn main` ends, libstd's runtime calls `exit(0)`.
//!   That is the only family in 212.N.2 where `run_entry` does not
//!   diverge — POSIX hands `exit_success` / `_failure` off to libc,
//!   FreeRTOS / ThreadX never let `main` return at all, but NuttX's
//!   shell dispatch reclaims the task on a normal return. Returning
//!   the `Result` keeps it observable to a hosted test harness.
//! - No transport-bringup / network-wait step. NuttX brings up
//!   `eth0` (virtio-net etc.) during kernel boot before `main`
//!   runs; `init_hardware` re-applies IP overrides (qemu-arm overlay
//!   uses `SIOCSIFADDR`) and the 5 s sleep at the top of `run_entry`
//!   covers the virtio-net link-up race documented in `node::run`.
//!
//! ## SDK env-var contract
//!
//! NuttX owns the kernel build; the Cargo side reads:
//!
//! | Var | Purpose |
//! |---|---|
//! | `NUTTX_DIR` | Source root for header discovery (used by `nros-platform-cffi`'s NuttX C port). |
//!
//! Compared to FreeRTOS / ThreadX scaffolds, no kernel-source /
//! port-dir / config-dir env vars are read here. NuttX's own
//! `make menuconfig` + `defconfig` flow drives all of that.

// phase-359 W7 — `no_std`, unconditionally.
//
// This used to read `cfg_attr(not(any(feature = "reference-qemu", target_os =
// "nuttx")), no_std)`: std when the target was NuttX, `no_std` otherwise. The
// predicate existed because the bodies below reached for `std::io::stdout`,
// `std::thread` and `std::process::exit`, so the crate's FLAVOUR had to follow
// whether those bodies were live. None of them reach for std any more — every
// facility they used is exported by `<nros/platform.h>` or NuttX's own libc,
// both of which this image already links — so the predicate has nothing left to
// decide and the crate is one flavour on every target.
//
// See `docs/roadmap/phase-359-drop-std-campaign.md` W7 for what each std
// facility became.
#![no_std]

extern crate alloc;

/// phase-359 W7 — the NuttX system facilities this board used to reach through
/// `std`.
///
/// Every one of them is a call NuttX already exports: the console and `exit`
/// come from its libc (which is what libstd called underneath), and `sleep`
/// comes from the canonical platform ABI this image links as
/// `libnros_platform_nuttx.a`. Nothing here is new capability — it is the same
/// syscall, reached without compiling the standard library to get to it.
#[doc(hidden)]
pub mod sys {
    use core::ffi::{c_char, c_int, c_void};

    unsafe extern "C" {
        /// The console. `println!` bottomed out here through libstd's
        /// `Stdout` -> `LineWriter` -> `write(2)`; this is the same fd and the
        /// same syscall with the two wrappers removed.
        fn write(fd: c_int, buf: *const c_void, count: usize) -> isize;
        /// What libstd's `process::exit` calls. Same status semantics, so a
        /// caller of `BoardExit::exit_failure` is unaffected.
        fn exit(code: c_int) -> !;
        /// `<nros/platform.h>` — the POSIX platform port NuttX already links.
        fn nros_platform_sleep_s(s: usize);
        /// Ditto — what `std::thread::yield_now` called.
        fn nros_platform_yield_now();
    }

    /// Write every byte of `bytes` to fd 1, tolerating short writes.
    fn write_all(bytes: &[u8]) {
        let mut off = 0usize;
        while off < bytes.len() {
            // SAFETY: `bytes[off..]` is a valid readable range for `len - off`.
            let n = unsafe {
                write(
                    1,
                    bytes.as_ptr().add(off) as *const c_void,
                    bytes.len() - off,
                )
            };
            if n <= 0 {
                // EOF or a hard error — a retry loop here would spin forever on
                // a closed console, and a diagnostic printer must never
                // out-live the thing it is diagnosing.
                return;
            }
            off += n as usize;
        }
    }

    /// A whole formatted line, assembled before it reaches the console.
    ///
    /// The buffer is not an optimisation, it is the ATOMICITY the std path
    /// provided: `println!` locked `Stdout` and its `LineWriter` emitted one
    /// `write(2)` per line, so two tiers printing concurrently could not
    /// interleave mid-line. Writing each `core::fmt` fragment straight to fd 1
    /// would have shredded exactly the multi-tier diagnostics that issues 0572
    /// and 0579 added — a tier's identity line arrives interleaved with
    /// another's, and the console stops being readable at the moment it matters.
    ///
    /// A line longer than the buffer degrades to several writes rather than
    /// being truncated: losing atomicity on an over-long line is recoverable,
    /// losing its tail is not.
    struct LineBuf {
        buf: [u8; 512],
        len: usize,
    }

    impl LineBuf {
        const fn new() -> Self {
            Self {
                buf: [0; 512],
                len: 0,
            }
        }
        fn flush(&mut self) {
            if self.len > 0 {
                write_all(&self.buf[..self.len]);
                self.len = 0;
            }
        }
    }

    impl core::fmt::Write for LineBuf {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            for &b in s.as_bytes() {
                if self.len == self.buf.len() {
                    self.flush();
                }
                self.buf[self.len] = b;
                self.len += 1;
            }
            Ok(())
        }
    }

    /// `println!`'s body — format, append the newline, emit as one write.
    pub fn print_line(args: core::fmt::Arguments<'_>) {
        use core::fmt::Write as _;
        let mut line = LineBuf::new();
        let _ = line.write_fmt(args);
        let _ = line.write_str("\n");
        line.flush();
    }

    /// Terminate the task, exactly as `std::process::exit` did.
    pub fn exit_process(code: i32) -> ! {
        // SAFETY: libc `exit` is always callable and diverges.
        unsafe { exit(code as c_int) }
    }

    /// Sleep whole seconds via the platform ABI.
    pub(crate) fn sleep_secs(s: usize) {
        // SAFETY: documented, always callable.
        unsafe { nros_platform_sleep_s(s) }
    }

    /// Yield the CPU via the platform ABI.
    pub(crate) fn yield_now() {
        // SAFETY: documented, always callable.
        unsafe { nros_platform_yield_now() }
    }

    /// Spawn a detached tier task with an explicit stack size (issue 0246).
    ///
    /// phase-364 W3 — this now goes through the ABI's own `task_init`, with a
    /// `nros_platform_task_attr_t`. It used to call `nros_nuttx_spawn_tier`, a
    /// bespoke C shim in `nuttx_run_tiers.c`, written for one reason: the ABI
    /// had no portable way to ask for a stack size, and building a
    /// `pthread_attr_t` from Rust is issue 0570 (a 20-byte Rust mirror met
    /// NuttX's 56-byte struct and `pthread_attr_init` smashed 36 bytes of the
    /// caller's frame). The attribute struct is the ABI's now, so the shim has
    /// nothing left to do — and the layout that must not be mirrored stays
    /// inside the port, which is where it always belonged.
    ///
    /// Returns 0 on success, else the platform return code (`NOMEM` for a
    /// transient refusal — issue 0246's case — which the caller retries).
    ///
    /// `priority` is the tier's DECLARED NuttX priority (0 = undeclared). It is
    /// passed in the attribute rather than left to the tier's self-apply at
    /// entry, because those two are not equivalent under SCHED_FIFO: a task
    /// born with `INHERIT` starts at the SPAWNING tier's priority, and the
    /// spawner here is the boot tier — the highest-priority one. On a
    /// single-core guest an equal-priority FIFO peer does not preempt, so the
    /// new tier ran only when the boot tier's spin happened to block, and its
    /// self-apply (with the marker RFC-0052 requires) was reached late or not
    /// within the cell's window. Measured 1 of 5 runs passing before this,
    /// which read as a flake and was a priority inversion for the whole
    /// interval between spawn and self-apply.
    pub(crate) fn spawn_tier(
        name: &str,
        stack_bytes: usize,
        priority: i64,
        entry: unsafe extern "C" fn(*mut c_void) -> *mut c_void,
        arg: *mut c_void,
    ) -> i32 {
        /// `NROS_PLATFORM_PRIORITY_RAW(n)` from `<nros/platform.h>` — the band's
        /// escape hatch for "this number is already in the kernel's own units",
        /// which a `[tiers.<name>.nuttx] priority` is by construction.
        const fn raw(n: i32) -> i32 {
            -0x4000_0000 - n
        }
        // `TierSpec::priority` is `i64` (one field for every kernel's range);
        // NuttX's is 1..=255, so anything outside that is not a priority this
        // port can express and falls back to INHERIT rather than wrapping into
        // some other tier's band.
        let priority = i32::try_from(priority)
            .ok()
            .filter(|p| (1..=255).contains(p));
        /// Mirrors `nros_platform_task_attr_t` from `<nros/platform.h>`.
        #[repr(C)]
        struct TaskAttr {
            name: *const c_char,
            stack_bytes: usize,
            stack_mem: *mut c_void,
            priority: i32,
            core: i8,
            flags: u8,
        }
        const DETACHED: u8 = 0x01;

        unsafe extern "C" {
            fn nros_platform_task_init(
                task: *mut c_void,
                attr: *mut c_void,
                entry: unsafe extern "C" fn(*mut c_void) -> *mut c_void,
                arg: *mut c_void,
            ) -> i8;
            fn nros_platform_task_storage_size() -> usize;
            fn nros_platform_task_storage_align() -> usize;
        }

        // Same NUL-terminated stack copy the `apply_tier_*` shims use — a
        // `TierSpec` name is a `&str`, never a C string.
        let mut name_buf = [0u8; 64];
        let n = name.len().min(63);
        name_buf[..n].copy_from_slice(&name.as_bytes()[..n]);

        // Task storage is sized by the port, never guessed here.
        // SAFETY: documented pure probes, callable before any task exists.
        let (size, align) = unsafe {
            (
                nros_platform_task_storage_size(),
                nros_platform_task_storage_align(),
            )
        };
        let Ok(layout) = core::alloc::Layout::from_size_align(size.max(1), align.max(1)) else {
            return -7; // INVALID
        };
        // SAFETY: non-zero size.
        let storage = unsafe { alloc::alloc::alloc(layout) };
        if storage.is_null() {
            return -6; // NOMEM
        }

        let mut attr = TaskAttr {
            name: name_buf.as_ptr() as *const c_char,
            stack_bytes,
            stack_mem: core::ptr::null_mut(),
            // `i32::MIN` is `NROS_PLATFORM_PRIORITY_INHERIT`.
            priority: match priority {
                Some(p) => raw(p),
                None => i32::MIN,
            },
            core: -1,
            flags: DETACHED,
        };
        // SAFETY: `storage` is the size/alignment the port asked for; `attr`
        // and `name_buf` outlive the call (the port copies what it keeps);
        // `entry`/`arg` are the caller's contract.
        let rc = unsafe {
            nros_platform_task_init(
                storage as *mut c_void,
                (&raw mut attr) as *mut c_void,
                entry,
                arg,
            )
        };
        if rc != 0 {
            // SAFETY: no task took the storage.
            unsafe { alloc::alloc::dealloc(storage, layout) };
        }
        // The task is DETACHED and spins forever, so its storage is
        // deliberately leaked on success — there is no point at which freeing
        // it would be correct.
        rc as i32
    }
}

/// Console line printer.
///
/// Shadows `std::println!` for the rest of this crate, with the same surface
/// and the same bytes on the wire, so no call site below changed when the crate
/// left `std`. The three `no_std` sibling boards
/// (`nros-board-mps2-an385{,-freertos}`, `nros-board-threadx-qemu-riscv64`)
/// each define this same macro over their own console primitive; NuttX's
/// primitive is `write(2)`.
///
/// Unbuffered by construction: one line, one syscall. That retires the ~25
/// explicit `stdout().flush()` calls the std path needed — with `LineWriter`
/// gone there is no userspace buffer left to strand a diagnostic in, which is
/// the failure mode issue 0572 was chasing.
#[macro_export]
macro_rules! nros_nuttx_println {
    () => { $crate::sys::print_line(format_args!("")) };
    ($($arg:tt)*) => { $crate::sys::print_line(format_args!($($arg)*)) };
}

/// Crate-local spelling, so no call site below had to change.
///
/// Unused off-target: every caller sits behind the `reference-qemu` /
/// `target_os = "nuttx"` gate, which a host build (e.g. `nros sync`'s
/// source-metadata probe) does not satisfy.
#[allow(unused_macros)]
macro_rules! println {
    ($($arg:tt)*) => { $crate::nros_nuttx_println!($($arg)*) };
}

// Phase 313 W-nuttx (#0243) — the legacy `nros_board_common::board_init` path is
// RETIRED for the NuttX family: the generic `run_generic<B>` shim, the
// `nros_board_common::BoardInit` re-export it consumed, and the `reference-qemu`
// scaffolding re-export of the per-board free `run` are all gone. The live entries
// are the `nros_platform`-shaped `run_entry` / `run_tiers` below (consumed by
// `nros::main!` via each board's `impl nros_platform::BoardEntry`).

/// Phase 212.N.2 — `BoardEntry`-shaped NuttX entry point.
///
/// Mirrors the [`nros_platform::BoardEntry::run`] signature so the
/// Phase 212.N.4 codegen-emitted Entry pkg `main.rs` can call into
/// the NuttX family driver without owning a [`Config`]:
///
/// ```ignore
/// use nros_board_nuttx::run_entry;
/// use nros_board_nuttx_qemu::NuttxQemu;
///
/// fn main() -> Result<(), MyError> {
///     run_entry::<NuttxQemu, _, _>(|runtime| {
///         // codegen-emitted (Phase 212.N.4)
///         run_plan(runtime)
///     })
/// }
/// ```
///
/// ## Lifecycle
///
/// 1. [`nros_platform::BoardInit::init_hardware`] (no-arg variant
///    from the 212.N.1 trait family — distinct from the legacy
///    [`nros_board_common::BoardInit::init_hardware`] which takes a
///    `&Config`). Per-board overlay state, if any, lives inside `B`'s
///    impl block.
/// 2. 5-second NuttX virtio-net warm-up — kernel `NETINIT_*` runs
///    synchronously before `main`, but link-up isn't atomic;
///    `connect_timeout` doesn't observe a partially-up interface.
///    Same magic number `run` / `run_generic` use.
/// 3. Flush stdout (NuttX line-buffers around `write(2)`).
/// 4. Build a [`nros_platform::RuntimeCtx`]. Today this is the
///    [`nros_platform::RuntimeCtx::with_runtime`] placeholder; Phase 212.N.4
///    codegen will populate `params` / `remaps` / `env` from the
///    launch overlay + `--ros-args` CLI parsing.
/// 5. Invoke `setup(&mut runtime)` and **return its result**.
///
/// ## Why this does not diverge
///
/// Sibling family drivers in 212.N.2 each diverge into
/// `BoardExit::exit_*`:
///
/// - `nros-board-linux` calls `std::process::exit(0|1)` —
///   libstd's runtime hands the integer to `_exit(2)`.
/// - `nros-board-freertos` traps in an infinite loop — the FreeRTOS
///   scheduler never permits `main` to return.
/// - `nros-board-threadx` traps similarly — `tx_kernel_enter` never
///   returns.
///
/// NuttX is the carve-out: the shell's task-dispatch loop spawns the
/// application via `task_create` (or `nsh` builtin dispatch) and
/// reclaims the task when its entry returns, exactly like a normal
/// POSIX `main`. Returning the [`Result`] (rather than collapsing to
/// `!` via `exit`) keeps the application status observable to a
/// hosted test harness that wants to drive `run_entry` without
/// killing the test process.
///
/// Production NuttX targets typically pair `run_entry` with the
/// usual `fn main() -> Result<…>` shape; the libstd runtime's
/// `lang_start` then maps `Ok(())` → exit-status-0 and `Err(_)` →
/// exit-status-1 on return, so the user observes the same exit
/// semantics as the diverging siblings.
///
/// ## SDK availability
///
/// Compiled only when `std` is reachable — gated on the same
/// `reference-qemu` / `target_os = "nuttx"` predicate as
/// [`run_generic`] so a bare `cargo check` without a NuttX target
/// + without the reference feature skips this body. The `run_entry`
/// symbol therefore only exists in builds that can actually call it.
/// Route panics to STDOUT (issue 0572; extended to `run_tiers` by issue 0583).
///
/// A panic on this guest is INVISIBLE otherwise: Rust prints the message and
/// location to stderr, and stderr does not reach the NuttX serial console here
/// — the same finding that hid every `eprintln!` diagnostic on this board. Its
/// own comment said "a boot tier that panics after spawning its siblings would
/// look exactly like one that silently stopped scheduling", and it was then
/// installed on ONE of the two entry paths: `run_tiers` — the multi-tier path,
/// the only one that HAS siblings to spawn — never called it. Issue 0583 is
/// exactly the scenario it describes, on exactly the path that lacked it.
///
/// ## phase-359 W7 — a hook became a handler
///
/// This was `std::panic::set_hook`, installed at the top of each entry. A
/// `no_std` image has no panic RUNTIME to hook, so the same job is now a
/// `#[panic_handler]`: it runs for every panic in the image without an entry
/// having to remember to install it, which structurally retires the 0583 class
/// (an entry path that forgot the call). It also cannot be displaced by a later
/// `set_hook`, so a user crate cannot silently take these diagnostics away.
///
/// The message text is deliberately byte-identical to the hook's
/// (`nros: PANIC <info>`) — the e2e harness greps for it.
///
/// Diverging is now the handler's job rather than the runtime's. The target
/// spec already says `panic-strategy: abort` (and the profiles set
/// `panic = "abort"`), so nothing unwinds past here either way; `exit(1)` keeps
/// the status a NuttX shell observes identical to what libstd produced when its
/// abort path ran.
///
/// ## Why gated
///
/// Exactly one `#[panic_handler]` may exist per image, and `nros-c` supplies
/// one for `no_std` C/C++ images (its own gate: `global-allocator`, not `std`,
/// not `panic-halt`). Both crates are linked into a C/C++ NuttX image, so the
/// two would be a duplicate-lang-item link error. Those images therefore take
/// this crate with `default-features = false` and let `nros-c` own the image
/// runtime; a pure-Rust image links no `nros-c` and takes this handler. See the
/// `image-runtime` feature in `Cargo.toml` for why the handler and the
/// allocator share one flag.
/// The board's fatal behaviour, as a STRONG `nros_platform_panic` (phase-366
/// W5), overriding the weak default in `nros-platform-posix` (stderr +
/// `abort()`). `exit(1)` rather than `abort()` is the point: a NuttX shell sees
/// the status libstd's abort path produced, which is what the e2e harness
/// expects.
///
/// The rendered line stays byte-identical — `nros: PANIC <info>` — because the
/// harness greps for it.
#[cfg(all(target_os = "nuttx", feature = "image-runtime"))]
#[unsafe(no_mangle)]
pub extern "C" fn nros_platform_panic(msg: *const u8, len: usize) -> ! {
    let text = if msg.is_null() || len == 0 {
        ""
    } else {
        // SAFETY: the ABI contract is `len` readable bytes at `msg`.
        core::str::from_utf8(unsafe { core::slice::from_raw_parts(msg, len) })
            .unwrap_or("<non-utf8>")
    };
    println!("nros: PANIC {text}");
    sys::exit_process(1)
}

// phase-366 W5.d — the `#[panic_handler]` that lived here is GONE.
//
// A library must not claim a singleton of the final artifact (RFC-0077). The
// BEHAVIOUR stays, above, as this board's strong `nros_platform_panic`; the lang
// item belongs to the image, which writes `nros::panic_to_platform!()` in its
// entry (or its own handler).
//
// This also retires the doc-comment rule that used to live here — "C/C++ images
// take this crate with `default-features = false` and let `nros-c` own the image
// runtime; a pure-Rust image links no `nros-c` and takes this handler". That was
// a composition rule in PROSE, correct only if every consumer knew which shape
// of image it was building. Nothing checked it. Now nobody has to know: the
// image says what it wants, once.

// phase-359 W7 — `run_entry` carries the same gate as `run_tiers` and as every
// helper both of them call (`install_stdout_logger`, `nuttx_run_one_tier`,
// `nuttx_spin_tier_forever`, `NUTTX_TIER_STACK_DEFAULT_BYTES`).
//
// It did not before, and the reason is issue 0579's class again: the gate ONE
// line above this fn belonged to `install_stdout_panic_hook`, which sat between
// the two, so `run_entry` looked gated in context while being ungated in fact.
// Nothing noticed because a host build still had `std`, so the ungated body
// compiled; with the family on `no_std` it stops compiling off-target and its
// gated helpers vanish out from under it. The host build that surfaced this is
// `nros sync`'s source-metadata probe, which builds these leaves for the host.
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
pub fn run_entry<B, F, E>(
    boot_config: Option<&'static nros_platform::BakedBootConfig>,
    setup: F,
) -> Result<(), E>
where
    B: nros_platform::BoardInit,
    F: FnOnce(&mut nros_platform::RuntimeCtx<'_>) -> Result<(), E>,
    E: core::fmt::Debug,
{
    // issue 0708 — publish the nros_log sink list at the boot funnel.
    //
    // The board also installs a `log`-crate logger below, and that is a
    // DIFFERENT facade: an `log_error!` raised inside a LIBRARY (the zenoh
    // session-pool diagnostic of issue 0589, for one) dispatches through
    // nros_log, which drops every record until a sink list is published.
    // Measured before the fix: the threadx-linux logging fixture with its own
    // `init` removed booted fully and emitted 0 of 6 records.
    //
    // At the FUNNEL, not next to `install_uart_logger`/`install_stdout_logger`:
    // those live in inner helpers, and a first attempt that patched them left
    // `run_bare` — the funnel the fixture actually boots through — still
    // silent. Idempotent, so nesting funnels may each call it.
    ::nros_platform_cffi::log::init_default();
    <B as nros_platform::BoardInit>::init_hardware();

    // NuttX virtio-net needs a brief warm-up after kernel
    // `NETINIT_*` before `connect()` succeeds. Magic number matches
    // `run` / `run_generic`; future work could probe link state
    // via `SIOCGIFFLAGS` instead.
    sys::sleep_secs(5);

    // Phase 212.N.7 step-3.5 — open the executor + wrap it in an
    // `ExecutorNodeRuntime` so the codegen-emitted `run_plan(runtime)`
    // body can register components against a live RMW session.
    //
    // Locator/domain are baked at COMPILE time on NuttX, not read from
    // the runtime env. Although NuttX ships `std` + libc `getenv`, the
    // QEMU guest has no environment populated, so `from_env()` would
    // silently fall back to its loopback default (`tcp/127.0.0.1:7447`)
    // — the connection then never leaves the guest over virtio-net and
    // fails fast with `Transport(ConnectionFailed)`. Bake via
    // `option_env!` (the freertos/esp32 pattern; CLAUDE.md "compile-time
    // on embedded") and fall back to `from_env` only when nothing was
    // baked (hosted/dev use).
    const BAKED_LOCATOR: Option<&str> = option_env!("NROS_LOCATOR");
    const BAKED_DOMAIN: Option<&str> = option_env!("NROS_DOMAIN_ID");
    // Issue #98 / RFC-0045 — derive the node name from the baked boot config
    // supplied by `run_with_deploy`; fall back to `"nros_app"` when called from
    // `run` (boot_config = None) or when the baked config carries no name.
    // Hoisted out of the BAKED_LOCATOR match so the no-baked-locator path
    // (`from_env`) also applies the launch-declared node name (W4d fix).
    let node_name: &'static str = boot_config
        .map(::nros::BootConfig::from_baked)
        .and_then(|b| b.node_name)
        .unwrap_or("nros_app");
    let exec_cfg = match BAKED_LOCATOR {
        Some(loc) => {
            let mut cfg = ::nros::ExecutorConfig::new(loc).node_name(node_name);
            if let Some(d) = BAKED_DOMAIN.and_then(|s| s.parse::<u32>().ok()) {
                cfg = cfg.domain_id(d);
            }
            cfg
        }
        // phase-359 W7 — was `ExecutorConfig::from_env()`, which is std-only
        // (it reads `std::env`). Nothing is lost: this guest has NO populated
        // environment, which is exactly why the locator is baked above, and
        // issue 0330 defines an unset env as an EMPTY locator that the backend
        // then defaults. `new("")` is that same state, spelled without a
        // lookup that could only ever miss. (The one other field `from_env`
        // set, a std wall-clock epoch, is `None` in any no_std build anyway.)
        None => ::nros::ExecutorConfig::new("").node_name(node_name),
    };

    // Explicitly register the zenoh RMW backend before opening the executor.
    // The unified-RMW `nros_rmw_register_backend!` macro is a no-op on NuttX
    // (linkme has no NuttX support) and the flat image does not run the
    // auto-register `.init_array` path, so without this the CFFI vtable has
    // no transport and `Executor::open` fails with `Transport(ConnectionFailed)`.
    #[cfg(feature = "rmw-zenoh")]
    if let Err(err) = ::nros_rmw_zenoh::register() {
        println!("nros: zenoh RMW backend register failed: {:?}", err);
    }

    let executor = match ::nros::Executor::open(&exec_cfg) {
        Ok(e) => e,
        Err(err) => {
            println!("Executor::open failed: {:?}", err);
            sys::exit_process(1);
        }
    };
    // #132 — install a stdout `log::Log` sink so the chatter examples'
    // `log::info!("Publishing:" / "I heard:")` reach the console. The facade is
    // otherwise dark on NuttX, so pub/sub delivery was invisible to the e2e
    // harness even when it worked. Idempotent + before the readiness marker.
    install_stdout_logger();

    // #132 — stable boot-readiness marker. A subscriber-only entry
    // (`listener-entry`) prints nothing until it receives, so the rtos_e2e
    // harness had no line to gate "session up, node registered" on (the C
    // examples' "Waiting for messages" is C-only). Emit one after the session
    // opens and before spin — greppable. The pattern is a test contract.
    println!("nros entry ready");

    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(executor);
    let mut runtime = nros_platform::RuntimeCtx::with_runtime(&mut crt);
    let setup_result = setup(&mut runtime);

    if let Err(ref e) = setup_result {
        println!("Application error: {:?}", e);
        return setup_result;
    }

    // Phase 212.N.7 step-3.5 — embedded RTOS spin loop. NuttX is a
    // shell-dispatched POSIX-style hosted env: returning would have
    // the shell reclaim the task, so the application would stop
    // dispatching component callbacks. Spin forever like the FreeRTOS
    // / ThreadX siblings; the user terminates via signal or shell.
    loop {
        if let Err(err) = nros_platform::NodeDispatchRuntime::spin_once(&mut crt, 10) {
            println!("spin_once error: {:?}", err);
            sys::exit_process(1);
        }
    }
}

// phase-296 W5.9 — NuttX kernel sporadic server, self-applied on the
// CALLING thread. Defined in the board seam C file (`nuttx_run_tiers.c`,
// compiled by the board crate's build.rs) so `struct sched_param`'s
// config-gated sporadic fields are laid out per THIS kernel's config (the
// #131 layout-mirror trap avoided). Returns 1 when the kernel accepted
// SCHED_SPORADIC, 0 otherwise (no CONFIG_SCHED_SPORADIC / no policy
// declared / kernel rejection — the C side logs the marker or the loud
// fallback; the executor's cooperative Sporadic SchedContext remains the
// enforcement either way).
#[cfg(target_os = "nuttx")]
unsafe extern "C" {
    fn nros_nuttx_apply_current_sporadic(
        name: *const core::ffi::c_char,
        tier_class: *const core::ffi::c_char,
        budget_us: u64,
        period_us: u64,
        priority: i64,
    ) -> i32;
}

// phase-296 W5.11 — NuttX SMP core affinity (the placement dim), self-applied
// on the CALLING thread. Defined in the board seam C file (`nuttx_run_tiers.c`)
// so `cpu_set_t` / `pthread_setaffinity_np` lay out per THIS kernel's config
// (config-gated on CONFIG_SMP — the #131 layout trap avoided; Rust never
// mirrors the set). Returns 1 when the kernel accepted the pin, 0 otherwise
// (unpinned tier / no CONFIG_SMP / rejection — the C side logs the accept
// marker or the loud fallback note). The ABI carried `core_plus1` since W2 but
// had NO consumer before this — a declared `core` was silently dropped.
#[cfg(target_os = "nuttx")]
unsafe extern "C" {
    fn nros_nuttx_apply_current_affinity(name: *const core::ffi::c_char, core_plus1: u32) -> i32;
}

// phase-302 W3 (issue 0263) — the C shim that adopts the tier's declared
// SCHED_FIFO priority on the calling thread (nuttx_run_tiers.c, shared with
// the C arm's create-time path). std's Builder has no priority attr, so the
// Rust arm self-applies at tier entry — without this a non-sporadic tier ran
// at the parent's priority (invisible until contention).
unsafe extern "C" {
    fn nros_nuttx_apply_current_priority(name: *const core::ffi::c_char, priority: u32) -> i32;
}

/// Adopt the tier's declared priority on the current thread (no-op
/// off-target and when the tier declares none). Name crosses the FFI as a
/// NUL-terminated stack copy, mirroring [`apply_tier_affinity`].
#[cfg(target_os = "nuttx")]
fn apply_tier_priority(tier: &nros_platform::TierSpec<'_>) {
    if tier.priority <= 0 {
        return;
    }
    let mut name_buf = [0u8; 64];
    let n = tier.name.len().min(63);
    name_buf[..n].copy_from_slice(&tier.name.as_bytes()[..n]);
    unsafe {
        nros_nuttx_apply_current_priority(
            name_buf.as_ptr() as *const core::ffi::c_char,
            tier.priority as u32,
        );
    }
}

#[cfg(not(target_os = "nuttx"))]
#[inline]
fn apply_tier_priority(_tier: &nros_platform::TierSpec<'_>) {}

/// Default per-tier pthread stack for spawned Rust tiers (issue #246). Mirrors
/// the C glue's `NROS_NUTTX_TIER_STACK_BYTES` intent but sized at NuttX's own
/// `CONFIG_PTHREAD_STACK_DEFAULT` (64 KiB): the executor arena lives on the
/// heap (`nros_platform_alloc`), so this only carries the zenoh-pico/executor
/// call frames. `TierSpec::stack_bytes` (when non-zero) overrides it.
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
const NUTTX_TIER_STACK_DEFAULT_BYTES: usize = 65536;

/// Self-apply the tier's kernel sporadic policy (no-op off-target and when
/// the tier declares no budget/period). Name/class cross the FFI as
/// NUL-terminated stack copies (TierSpec strings are `&str`, not C strings).
#[cfg(target_os = "nuttx")]
fn apply_tier_sporadic(tier: &nros_platform::TierSpec<'_>) {
    let (Some(class), Some(budget), Some(period)) = (tier.class, tier.budget_us, tier.period_us)
    else {
        return;
    };
    let mut name_buf = [0u8; 64];
    let n = tier.name.len().min(63);
    name_buf[..n].copy_from_slice(&tier.name.as_bytes()[..n]);
    let mut class_buf = [0u8; 32];
    let c = class.len().min(31);
    class_buf[..c].copy_from_slice(&class.as_bytes()[..c]);
    unsafe {
        nros_nuttx_apply_current_sporadic(
            name_buf.as_ptr() as *const core::ffi::c_char,
            class_buf.as_ptr() as *const core::ffi::c_char,
            budget,
            period,
            tier.priority,
        );
    }
}

#[cfg(not(target_os = "nuttx"))]
#[inline]
fn apply_tier_sporadic(_tier: &nros_platform::TierSpec<'_>) {}

/// phase-296 W5.11 — self-apply the tier's SMP core pin (no-op off-target and
/// when the tier declares no `core`). The `core + 1` encoding (0 = unpinned)
/// matches the C emit. Name crosses the FFI as a NUL-terminated stack copy.
/// Safe on the session-owning boot tier too: a core pin does not budget-cap the
/// thread, so (unlike the sporadic server, #246) it never starves the shared
/// session flush.
#[cfg(target_os = "nuttx")]
fn apply_tier_affinity(tier: &nros_platform::TierSpec<'_>) {
    let Some(core) = tier.core else {
        return;
    };
    let mut name_buf = [0u8; 64];
    let n = tier.name.len().min(63);
    name_buf[..n].copy_from_slice(&tier.name.as_bytes()[..n]);
    unsafe {
        nros_nuttx_apply_current_affinity(
            name_buf.as_ptr() as *const core::ffi::c_char,
            core.saturating_add(1),
        );
    }
}

#[cfg(not(target_os = "nuttx"))]
#[inline]
fn apply_tier_affinity(_tier: &nros_platform::TierSpec<'_>) {}

// issue 0579 / phase-358 W4 — this doc block and the `#[cfg]` below it were
// STRANDED ~150 lines above, before the `apply_tier_*` extern blocks that got
// inserted between them and this fn. Attributes bind to the NEXT item, so the
// cfg was guarding an `unsafe extern "C"` block and `run_tiers` — which uses
// `println!` and threads — was compiled UNCONDITIONALLY. It has not bitten
// because this crate is workspace-excluded and only ever built for NuttX. The
// rustdoc `unused doc comment` error on those extern blocks was the symptom.
/// phase-281 W3-nuttx (RFC-0015 Model 1) — per-tier multi-task NuttX entry.
///
/// The multi-tier sibling of [`run_entry`]: opens the ONE RMW session, then
/// runs one [`nros::Executor`] per [`nros_platform::TierSpec`] over that shared
/// session. NuttX ships `std` and its zenoh-pico build sets
/// `Z_FEATURE_MULTI_THREAD = 1` (`platforms/nuttx/nros-platform.toml`
/// `[platform.nuttx]`), so `std::thread` maps onto NuttX pthreads and this
/// mirrors the **native Linux** [`nros_board_linux`] `run_tiers` (a scoped
/// thread per tier over one session) rather than the FFI k_thread shim the
/// Zephyr / bare-metal boards need.
///
/// ## Ordering (issue #144 — the interest-handshake race)
///
/// zenoh-pico entity declares carry an interest handshake; two threads that
/// declare concurrently race it, and the losing publisher's write filter can
/// stay closed (every put silently dropped). To avoid it we run the **boot
/// tier's `setup` FIRST on the boot task** (its declares finish before any
/// other tier starts), THEN spawn the remaining tiers. A spawned tier's `setup`
/// overlaps only the boot tier's *spin* (keepalives / data, not declares) — the
/// two-tier demo is therefore race-free. (For the single-tier deploy the
/// byte-identical [`run_entry`] path is used instead.)
///
/// `setup` is `Fn` (invoked once per tier) + `Sync` (spawned tiers share
/// `&setup`); it must register entities only — this fn owns each tier's
/// `active_groups` filter + the spin loop. Blocks forever (the boot tier's spin
/// never returns); returns only if the boot tier's `setup` fails before spin.
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
pub fn run_tiers<B, F, E>(
    boot_config: Option<&'static nros_platform::BakedBootConfig>,
    tiers: &[nros_platform::TierSpec<'_>],
    setup: F,
) -> Result<(), E>
where
    B: nros_platform::BoardInit,
    F: Fn(&mut nros_platform::RuntimeCtx<'_>) -> Result<(), E> + Sync,
    E: core::fmt::Debug,
{
    // issue 0708 — publish the nros_log sink list at the boot funnel.
    //
    // The board also installs a `log`-crate logger below, and that is a
    // DIFFERENT facade: an `log_error!` raised inside a LIBRARY (the zenoh
    // session-pool diagnostic of issue 0589, for one) dispatches through
    // nros_log, which drops every record until a sink list is published.
    // Measured before the fix: the threadx-linux logging fixture with its own
    // `init` removed booted fully and emitted 0 of 6 records.
    //
    // At the FUNNEL, not next to `install_uart_logger`/`install_stdout_logger`:
    // those live in inner helpers, and a first attempt that patched them left
    // `run_bare` — the funnel the fixture actually boots through — still
    // silent. Idempotent, so nesting funnels may each call it.
    ::nros_platform_cffi::log::init_default();

    <B as nros_platform::BoardInit>::init_hardware();

    // NuttX virtio-net warm-up — same magic number + rationale as `run_entry`.
    sys::sleep_secs(5);

    if tiers.is_empty() {
        println!("nros: run_tiers called with no tiers — nothing to run");
        sys::exit_process(1);
    }

    // Baked locator / domain / node name — identical to `run_entry` (compile-time
    // on embedded; the QEMU guest has no populated env, so `from_env` would fall
    // back to loopback and never leave the guest). See `run_entry` for detail.
    const BAKED_LOCATOR: Option<&str> = option_env!("NROS_LOCATOR");
    const BAKED_DOMAIN: Option<&str> = option_env!("NROS_DOMAIN_ID");
    let node_name: &'static str = boot_config
        .map(::nros::BootConfig::from_baked)
        .and_then(|b| b.node_name)
        .unwrap_or("nros_app");
    let exec_cfg = match BAKED_LOCATOR {
        Some(loc) => {
            let mut cfg = ::nros::ExecutorConfig::new(loc).node_name(node_name);
            if let Some(d) = BAKED_DOMAIN.and_then(|s| s.parse::<u32>().ok()) {
                cfg = cfg.domain_id(d);
            }
            cfg
        }
        // phase-359 W7 — was `ExecutorConfig::from_env()`, which is std-only
        // (it reads `std::env`). Nothing is lost: this guest has NO populated
        // environment, which is exactly why the locator is baked above, and
        // issue 0330 defines an unset env as an EMPTY locator that the backend
        // then defaults. `new("")` is that same state, spelled without a
        // lookup that could only ever miss. (The one other field `from_env`
        // set, a std wall-clock epoch, is `None` in any no_std build anyway.)
        None => ::nros::ExecutorConfig::new("").node_name(node_name),
    };

    // NuttX has no linkme / `.init_array` auto-register, so the backend register
    // is explicit (mirrors `run_entry`).
    #[cfg(feature = "rmw-zenoh")]
    if let Err(err) = ::nros_rmw_zenoh::register() {
        println!("nros: zenoh RMW backend register failed: {:?}", err);
    }

    // The boot task opens the one session and owns it for the program's life
    // (the boot tier's spin loop never returns).
    let boot_exec = match ::nros::Executor::open(&exec_cfg) {
        Ok(e) => e,
        Err(err) => {
            println!(
                "nros: Executor::open failed ({:?}); multi-tier entry needs a live session \
                 — aborting.",
                err
            );
            sys::exit_process(1);
        }
    };
    install_stdout_logger();
    // Boot-readiness marker (same contract as `run_entry`) + a multi-tier marker
    // an E2E can gate on ("this image entered the per-tier run with a live
    // session"); the single-tier `run_entry` never prints the latter.
    println!("nros entry ready");
    println!(
        "nros: multi-tier run — {} tier(s) over one session",
        tiers.len()
    );

    let mut boot_crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(boot_exec);

    // issue #144 — boot-tier declares FIRST, before spawning any other tier.
    //
    // issue 0636 — the boot tier is CHOSEN, not `tiers[0]`. This board runs a
    // uniprocessor guest under SCHED_FIFO, and `resolve_tiers` orders by raw
    // number descending without inverting per kernel, so `tiers[0]` was the
    // MOST urgent tier here. An owner that outranks its peers and then spins
    // starves them — measured at 1 of 5 runs before a spawned tier reached its
    // first statement at all — and `sched_yield` cannot rescue it, because
    // under FIFO a yield rotates the caller within its OWN priority queue and
    // never lets a lower-priority thread run. `boot_tier_index` picks the tier
    // that outranks nothing.
    let boot_index =
        nros_platform::boot_tier_index(tiers, nros_platform::PriorityDirection::BiggerIsMoreUrgent);
    let boot_tier = &tiers[boot_index];
    // issue 0572 — say WHICH tier is the session-owning boot tier, and with what.
    // The boot tier is the one that prints no priority marker (it keeps the
    // inherited FIFO priority by design), so the console showed the spawned
    // tiers only and the reader could not tell whether tiers[0] was the tier
    // they meant, nor whether its knobs survived the bake. On STDOUT with the
    // rest: this guest's stderr does not reach the serial console.
    println!(
        "nros: boot tier `{}` (session owner) — groups {:?}, class {:?}, \
         budget {:?} us, period {:?} us, spin {} us, priority {}",
        boot_tier.name,
        boot_tier.groups,
        boot_tier.class,
        boot_tier.budget_us,
        boot_tier.period_us,
        boot_tier.spin_period_us,
        // issue 0579 — the EFFECTIVE priority, so "accepted and dropped" is
        // visible from the console instead of needing a crash dump's tier
        // table to notice. `0` means the tier declared none and the thread
        // keeps whatever the init task was started with.
        boot_tier.priority
    );
    boot_crt.executor_mut().set_active_groups(boot_tier.groups);
    // W5.4 — shared tier→SchedContext lowering (Sporadic / EDF / TT). BUT the
    // boot tier is the SESSION OWNER: `apply_tier_sched_policy` installs the
    // lowered context as the executor's *default* SchedContext, which gates
    // EVERY dispatch on this executor — including the spin loop that flushes
    // the one shared zenoh-pico session for all tiers. A budget/sporadic policy
    // there caps the session flush and starves delivery (issue #246: the
    // high/ctrl publisher delivered exactly ONE sample). Unlike the Rust
    // default-SC model, the C++ path binds the lowered context per HANDLE, so
    // its boot-tier flush stays Fifo — which is exactly why the C/C++ siblings
    // pass with the same model. Mirror that: the session-owning boot tier keeps
    // the default Fifo SchedContext (drop its budget/period), so only the EDF
    // deadline dim — which does not gate throughput — can still lower here.
    let boot_is_budgeted = boot_tier.class == Some("real_time")
        && boot_tier.budget_us.is_some()
        && boot_tier.period_us.is_some();
    boot_crt.apply_tier_sched_policy(
        boot_tier.class,
        if boot_is_budgeted {
            None
        } else {
            boot_tier.period_us
        },
        if boot_is_budgeted {
            None
        } else {
            boot_tier.budget_us
        },
        boot_tier.deadline_us,
        boot_tier.deadline_policy,
    );
    // phase-296 W5.9 / issue #246 — likewise DON'T self-apply the kernel
    // SCHED_SPORADIC server to the boot tier here (nor below): it would drop
    // this session-owning thread to `sched_ss_low_priority` when its budget is
    // spent, stalling the shared flush. Spawn every tier at the boot tier's
    // normal FIFO priority; spawned NON-owner tiers self-apply the budget dim's
    // kernel + cooperative realization in `nuttx_run_one_tier`.
    {
        let mut ctx = nros_platform::RuntimeCtx::with_runtime(&mut boot_crt);
        if let Err(e) = setup(&mut ctx) {
            // issue 0572 — STDOUT. This guest's stderr does not reach the serial
            // console, so every `println!` diagnostic in this function has been
            // invisible: a boot-tier setup failure, a failed tier spawn, a spin
            // error. Issue 0565 taught the harness to capture the console for
            // exactly these lines, and they could never appear in it.
            println!("nros: boot tier `{}` setup FAILED: {:?}", boot_tier.name, e);
            return Err(e);
        }
    }

    let shared = NuttxSharedSession(boot_crt.executor_mut().session_ptr());
    let setup = &setup;
    {
        // Spawn every non-boot tier; each borrows the shared session pointer +
        // `&setup`. The boot declares are already done, so these only overlap the
        // boot tier's spin.
        // issue 0636 — the owner keeps its INHERITED priority through the spawn
        // loop and adopts its own (least urgent) one after it, at the call
        // below. Applying it here instead was measured WORSE — 4 of 8 against
        // 6 of 8 — and the failing console stopped dead after the owner's own
        // marker, before it had spawned anything: dropping to the least urgent
        // priority while the tier topology does not yet exist puts the owner
        // below the transport and system tasks that are already running, and
        // it never gets the CPU back to do the spawning. That is issue 0623's
        // hazard (a tier priority and a transport priority in one scheduler)
        // reached from the other side.
        for (spawn_index, tier) in tiers.iter().enumerate() {
            if spawn_index == boot_index {
                continue; // this one runs on the boot task
            }
            // issue 0572 — the spawned tiers' identity + groups, same reason.
            println!(
                "nros: spawning tier `{}` — groups {:?}, class {:?}, spin {} us",
                tier.name, tier.groups, tier.class, tier.spin_period_us
            );
            // issue #246 — a tier spawned with no explicit stack requested the
            // std default (2 MiB), which `pthread_create` cannot satisfy from
            // NuttX's small kernel heap → ENOMEM ("failed to spawn tier"). The
            // C/C++ sibling glue always passes an explicit
            // `pthread_attr_setstacksize` (16 KiB default / `stack_bytes`
            // override) precisely because the executor arena lives on the heap
            // (`nros_platform_alloc`), so a tier stack only carries call frames.
            // Mirror that: honour `stack_bytes`, else a 64 KiB default
            // (== NuttX's own `CONFIG_PTHREAD_STACK_DEFAULT`, generous for the
            // zenoh-pico/executor call depth the Rust closures reach).
            //
            // phase-359 W7 — the size is now handed to the C shim, which sets it
            // on a `pthread_attr_t` laid out by THIS kernel's headers. Same
            // number, same effect, one fewer std-shaped layer in between.
            let stack_bytes = if tier.stack_bytes > 0 {
                tier.stack_bytes
            } else {
                NUTTX_TIER_STACK_DEFAULT_BYTES
            };
            // phase-359 W7 — the per-tier context the spawned task receives.
            //
            // `thread::scope` used to carry the borrows (`&setup`, `tier`, the
            // shared session) into the spawned closure and PROVE they outlived
            // it. A detached NuttX task cannot be given that proof by the type
            // system, so the invariant becomes explicit and is stated here:
            // `run_tiers` never returns — the boot tier's spin loop below is
            // infinite — so every borrow reachable from this frame outlives
            // every task spawned from it. That is the same invariant the C arm
            // relies on (`nuttx_run_tiers.c` heap-allocates its ctx and never
            // frees it), now with the same lifetime and one owner.
            //
            // The box is deliberately leaked: the task never exits, so there is
            // no point at which freeing it would be correct.
            let ctx = alloc::boxed::Box::new(TierCtx::<F, E> {
                session: shared.0,
                tier: tier as *const nros_platform::TierSpec<'_>
                    as *const nros_platform::TierSpec<'static>,
                setup: setup as *const F,
                _e: core::marker::PhantomData,
            });
            // Keep the TYPED pointer as well: the `*mut c_void` the C shim wants
            // has lost the type needed to reconstruct the box on the failure path.
            let ctx_typed = alloc::boxed::Box::into_raw(ctx);
            let ctx_ptr = ctx_typed as *mut core::ffi::c_void;
            // issue #246 — NuttX `pthread_create` can fail TRANSIENTLY under
            // host/QEMU load, distinct from the deterministic 2-MiB-stack ENOMEM
            // the explicit stack size above fixes. A single failure drops the
            // whole tier for the run → the low tier never delivers → the cell
            // times out (the historical #246 flake). Retry a few times with a
            // yield between attempts.
            const SPAWN_ATTEMPTS: u32 = 5;
            let mut spawned = false;
            for attempt in 1..=SPAWN_ATTEMPTS {
                let rc = sys::spawn_tier(
                    tier.name,
                    stack_bytes,
                    tier.priority,
                    nuttx_tier_trampoline::<F, E>,
                    ctx_ptr,
                );
                if rc == 0 {
                    spawned = true;
                    break;
                }
                // W7 — this reports the pthread errno directly. The std path
                // could only offer `io::Error`, and issue 0246 recorded its
                // diagnosis as "an `io::Error` with no OS errno": std had
                // discarded the one number that identifies the failure.
                println!(
                    "nros: spawn tier `{}` attempt {}/{} failed (stack {} B, errno {})",
                    tier.name, attempt, SPAWN_ATTEMPTS, stack_bytes, rc
                );
                sys::yield_now();
            }
            if !spawned {
                // Reclaim the context no task took ownership of.
                // SAFETY: every attempt failed, so no task received `ctx_ptr`
                // and this is the only live pointer to that allocation.
                drop(unsafe { alloc::boxed::Box::from_raw(ctx_typed) });
                println!(
                    "nros: FAILED to spawn tier `{}` after {} attempts — tier will not run",
                    tier.name, SPAWN_ATTEMPTS
                );
            }
        }
        // phase-296 W5.9 / issue #246 — the boot tier is the SESSION OWNER:
        // its spin drives the one shared zenoh-pico session's TX flush for
        // EVERY tier (all spawned tiers borrow this session). A kernel
        // SCHED_SPORADIC server would drop this thread to `sched_ss_low_priority`
        // (== `SCHED_FIFO` min, prio 1) the moment its budget is spent, stalling
        // the whole session's flush and starving delivery on all tiers (observed:
        // the high/ctrl publisher delivered exactly ONE sample, then the flush
        // stalled). So the session-owning boot tier stays SCHED_FIFO — it is
        // NEVER budget-capped. The budget dim's kernel-Native realization applies
        // to NON-owner tiers, which self-apply it in `nuttx_run_one_tier`.
        if boot_is_budgeted {
            println!(
                "nros: tier `{}` declares a sporadic budget but is the session-owning \
                 boot tier — kept SCHED_FIFO (a kernel/cooperative budget cap would \
                 stall the shared session flush; non-owner tiers realize the budget)",
                boot_tier.name
            );
        }
        // phase-296 W5.11 — placement dim: the boot tier self-pins to its
        // declared `core` (safe here — a core pin, unlike the sporadic budget
        // above, does not cap CPU so it cannot starve the shared flush).
        apply_tier_affinity(boot_tier);
        // issue 0579 — and its declared PRIORITY, for exactly the reason the
        // affinity call above gives. `apply_tier_priority` was called from
        // `nuttx_run_one_tier` only, so tiers[0] parsed its
        // `[tiers.<name>.nuttx] priority`, baked it into the TierSpec, carried
        // it to the board and dropped it.
        //
        // Dropping one number out of an ORDERING does not make that tier
        // "default" — it silently reorders the set: a spawned tier declaring
        // 105 outranks a boot tier that declared 110, the inverse of what the
        // author wrote, with no diagnostic. It lands on the worst tier to get
        // wrong, since the boot tier is the session owner whose spin drives the
        // shared zenoh-pico flush.
        //
        // This is NOT issue 0246. That rule keeps the kernel SPORADIC SERVER
        // off the session owner because a spent budget drops it to
        // `sched_ss_low_priority` and stalls the shared flush — a mechanism
        // that CAPS CPU. A plain `pthread_setschedparam` priority caps nothing,
        // which is the same distinction the affinity comment draws, and
        // `boot_is_budgeted` above keeps the budget off the owner independently.
        //
        // ThreadX takes this answer too (`nros_threadx_set_current_priority`,
        // whose comment names the same inversion); Zephyr takes the other one,
        // sorting so tiers[0] is the numerically-largest = lowest-priority tier
        // and never needs to outrank anything (issue 0251). Two answers exist;
        // this board now has one of them rather than neither.
        // issue 0636 — the `yield_now()` run that stood here is gone. It existed
        // to hand the CPU to tiers the owner was about to OUTRANK, and the
        // owner does not outrank them any more: a more urgent tier preempts the
        // moment it is runnable. Under SCHED_FIFO a yield never lets a
        // lower-priority thread run in any case, which is why those yields
        // moved the rate to 4 of 6 and then stopped moving it.
        //
        // The owner adopts its own priority HERE — after spawning, before
        // spinning. Both neighbours were tried and measured: before the spawn
        // loop the owner starved against the transport tasks (4 of 8), and
        // never applying it at all leaves the marker RFC-0052 requires unset.
        apply_tier_priority(boot_tier);
        nuttx_spin_tier_forever(&mut boot_crt, boot_tier);
    }

    // Unreachable: the boot tier's spin loop never returns — which is also what
    // keeps every borrow the spawned tiers hold alive (see `TierCtx`).
    #[allow(unreachable_code)]
    Ok(())
}

/// phase-359 W7 — what a spawned tier receives, in place of a scoped closure's
/// captures.
///
/// Raw pointers rather than references because the value crosses a C `void *`
/// into a detached task, and a reference cannot survive that round trip with
/// its lifetime intact. Soundness rests on the invariant stated at the spawn
/// site: `run_tiers` never returns, so everything pointed at here outlives the
/// task.
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
struct TierCtx<F, E> {
    /// The boot executor's session, shared by every tier (see
    /// [`NuttxSharedSession`] for why sharing it is sound).
    session: *mut ::nros::internals::RmwSession,
    /// The tier's spec. Stored as `'static` because a raw pointer cannot carry
    /// the real lifetime; the invariant above is what makes reading it back
    /// sound.
    tier: *const nros_platform::TierSpec<'static>,
    /// The shared `setup` closure — `Fn` + `Sync`, invoked once per tier.
    setup: *const F,
    _e: core::marker::PhantomData<fn() -> E>,
}

/// C-ABI entry for a spawned tier: rebuild the borrows and run the tier.
///
/// Generic, so each `(F, E)` pair gets its own monomorphised entry — the
/// closure type is what a `void *` cannot carry.
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
unsafe extern "C" fn nuttx_tier_trampoline<F, E>(
    arg: *mut core::ffi::c_void,
) -> *mut core::ffi::c_void
where
    F: Fn(&mut nros_platform::RuntimeCtx<'_>) -> Result<(), E>,
    E: core::fmt::Debug,
{
    // SAFETY: `arg` is the `TierCtx` leaked by the spawn site, whose contents
    // outlive this task by the never-returns invariant documented there.
    let ctx = unsafe { &*(arg as *const TierCtx<F, E>) };
    // SAFETY: aliasing the boot executor's session is the per-tier model; the
    // backend serializes concurrent access internally (`Z_FEATURE_MULTI_THREAD`).
    let exec = unsafe { ::nros::Executor::open_with_session(ctx.session) };
    // SAFETY: both pointers came from live borrows at the spawn site.
    let (tier, setup) = unsafe { (&*ctx.tier, &*ctx.setup) };
    nuttx_run_one_tier::<F, E>(exec, tier, setup);
    core::ptr::null_mut()
}

/// `Send` wrapper for the shared raw session pointer so it can cross the
/// `std::thread::scope` boundary (the mirror of `nros-board-linux`'s
/// `SharedSession`). The pointed-to RMW session type is `pub(crate)` in
/// `nros-node`, so the wrapper is generic over `T` and never names it — `T` is
/// inferred from [`nros::Executor::session_ptr`]. Sharing the pointer is sound
/// under the per-tier contract: the boot executor owns the one session, the RMW
/// backend serializes concurrent access through its own locks (zenoh-pico
/// `Z_FEATURE_MULTI_THREAD = 1` on NuttX), and `thread::scope` guarantees no
/// spawned tier outlives the owner.
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
struct NuttxSharedSession<T>(*mut T);
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
impl<T> Clone for NuttxSharedSession<T> {
    fn clone(&self) -> Self {
        *self
    }
}
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
impl<T> Copy for NuttxSharedSession<T> {}
// SAFETY: the per-tier model shares one RMW session across tier tasks by design;
// concurrent access is serialized inside the backend.
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
unsafe impl<T> Send for NuttxSharedSession<T> {}

/// Register + spin one tier on a freshly-opened borrowed-session executor
/// (spawned-tier path).
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
fn nuttx_run_one_tier<F, E>(
    exec: ::nros::Executor<'static>,
    tier: &nros_platform::TierSpec<'_>,
    setup: &F,
) where
    F: Fn(&mut nros_platform::RuntimeCtx<'_>) -> Result<(), E>,
    E: core::fmt::Debug,
{
    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(exec);
    crt.executor_mut().set_active_groups(tier.groups);
    // W5.4 — shared tier→SchedContext lowering (Sporadic / EDF / TT).
    crt.apply_tier_sched_policy(
        tier.class,
        tier.period_us,
        tier.budget_us,
        tier.deadline_us,
        tier.deadline_policy,
    );
    // phase-302 W3 (issue 0263) — adopt the declared SCHED_FIFO priority
    // (std spawn carries no priority attr; the C arm sets it at create).
    apply_tier_priority(tier);
    // phase-296 W5.9 — kernel sporadic server for this tier thread, when declared.
    apply_tier_sporadic(tier);
    // phase-296 W5.11 — placement dim: SMP core pin for this tier, when declared.
    apply_tier_affinity(tier);
    {
        let mut ctx = nros_platform::RuntimeCtx::with_runtime(&mut crt);
        if let Err(e) = setup(&mut ctx) {
            // issue 0572 — STDOUT; this guest's stderr never reaches the console.
            println!(
                "nros: tier `{}` setup FAILED: {:?} — tier task exiting",
                tier.name, e
            );
            return;
        }
    }
    nuttx_spin_tier_forever(&mut crt, tier);
}

/// Drive a tier executor's `spin_once` at its declared period, forever.
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
fn nuttx_spin_tier_forever(
    crt: &mut ::nros::node_runtime::ExecutorNodeRuntime,
    tier: &nros_platform::TierSpec<'_>,
) {
    // issue 0572 — which wait the executor's spin will take. The primary
    // (session-owning) executor sleeps in the wake primitive when the backend
    // installed a wake callback; a borrowed one polls. `storage_size() == 0`
    // means no platform wake primitive is linked and the executor falls back to
    // a std `Condvar` — which on NuttX is the documented hang (this port's spin
    // is supposed to use `sem_timedwait`).
    unsafe extern "C" {
        fn nros_platform_wake_storage_size() -> usize;
    }
    // SAFETY: documented pure probe, callable before init.
    let wake_bytes = unsafe { nros_platform_wake_storage_size() };
    println!(
        "nros: tier `{}` entering spin — wake primitive {} ({} byte(s))",
        tier.name,
        if wake_bytes == 0 {
            "ABSENT (std Condvar fallback)"
        } else {
            "available"
        },
        wake_bytes
    );

    let period_ms = ((tier.spin_period_us / 1000).max(1)) as u32;
    // issue 0572 — a per-tier heartbeat carrying the counts `spin_once` used to
    // discard. A tier that dispatches NOTHING and a tier that is not running at
    // all look identical from outside the guest (a silent topic), and that
    // ambiguity is what left this cell undiagnosed. Once per ~5 s of spins, so
    // it costs one line per tier per five seconds on the serial console.
    // ~1 s, not 5: the e2e kills the guest a couple of seconds after the slow
    // tier's anchor, so a 5 s heartbeat never printed once.
    let heartbeat_every = (1_000_000 / tier.spin_period_us.max(1)).max(1);

    // issue 0736 — the discriminator the issue asks for, and it needs no host.
    //
    // The symptom is 1000 spins producing 7 fires of a 10 ms timer: the tier IS
    // scheduled, and its sense of time is not keeping up. Two candidates fit
    // that equally from outside — `spin_once` returning early without waiting
    // its declared period, or the clock the timer compares against
    // under-reporting — and the spin count alone cannot separate them.
    //
    // Reading the PLATFORM MONOTONIC clock at each heartbeat does: print the
    // clock delta beside the time those spins ASKED for. If they agree, the
    // spins really did take that little and the wait is the defect; if the
    // clock is far behind what was asked, the wait happened and the clock is.
    //
    // Deliberately the same clock the executor's timers compare against
    // (`nros_platform_clock_ns`, the platform ABI's monotonic source) — a
    // second, healthier clock would prove only that two clocks disagree.
    unsafe extern "C" {
        fn nros_platform_clock_ns() -> u64;
    }
    // SAFETY: a bare monotonic read, no arguments, defined by whichever
    // platform port linked this image — the same contract `nros-node` uses.
    let clock_ns = || unsafe { nros_platform_clock_ns() };
    let spin_start_ns = clock_ns();

    let mut iters: u64 = 0;
    let (mut timers, mut subs, mut errs) = (0usize, 0usize, 0usize);
    let mut announced_first = false;
    // issue 0636 option 3 — every iteration reaches a scheduling point. The
    // executor's own wait is SKIPPED whenever a wake already fired
    // (`spin_once` drives I/O with a ZERO timeout on that arm), so under
    // sustained traffic this loop would otherwise never block — and under
    // SCHED_FIFO, which this guest runs, a thread that never blocks never lets
    // a lower-priority tier run at all. `boot_tier_index` made the owner
    // outrank nothing, which is what fixed this issue; the gap is what makes
    // the guarantee hold without depending on that ordering being right.
    // Costs nothing while the spins do block.
    let mut gap = nros_platform::TierSpinGap::new(tier.spin_period_us);
    loop {
        let iter = gap.mark();
        match crt.spin_once_counted(core::time::Duration::from_millis(period_ms as u64)) {
            Ok(r) => {
                timers += r.timers_fired;
                subs += r.subscriptions_processed;
                errs += r.subscription_errors + r.service_errors;
            }
            Err(err) => {
                // STDOUT: this guest's stderr never reaches the serial console.
                println!("nros: tier `{}` spin error: {:?}", tier.name, err);
            }
        }
        gap.after_spin(iter);
        iters += 1;
        if iters == 1 {
            // The loop is ALIVE. Distinguishes "spinning but never dispatching"
            // from "never reached the spin at all" — two very different bugs
            // that both present as a silent topic.
            println!("nros: tier `{}` completed spin 1", tier.name);
        }
        // One-shot, and the datum that does not depend on how long the guest
        // lives: did this tier EVER dispatch anything?
        if !announced_first && (timers > 0 || subs > 0) {
            announced_first = true;
            println!(
                "nros: tier `{}` FIRST dispatch at spin {} — {} timer(s), {} sub callback(s)",
                tier.name, iters, timers, subs
            );
        }
        if iters % heartbeat_every == 0 {
            // issue 0736 — `clock` is what the platform monotonic source says
            // elapsed; `asked` is what those spins requested. A large gap
            // between them is the defect, and which side is wrong is the
            // question this line exists to answer.
            let clock_us = clock_ns().saturating_sub(spin_start_ns) / 1_000;
            let asked_us = iters.saturating_mul(tier.spin_period_us as u64);
            println!(
                "nros: tier `{}` alive — {} spin(s), {} timer(s) fired, {} sub callback(s), \
                 {} error(s), {} gap(s), clock {} us vs asked {} us",
                tier.name, iters, timers, subs, errs, gap.gaps(), clock_us, asked_us
            );
        }
    }
}

/// #132 — process-wide `log::Log` sink that writes each record to stdout as
/// `<message>` (the examples pre-format the level/prefix into the message
/// text). Installed once by [`run_entry`] so `log::info!` from the chatter /
/// service / action examples reaches the NuttX serial console; without it the
/// `log` facade drops every record on the floor (there is no default sink),
/// and the rtos_e2e harness could not observe pub/sub delivery even though the
/// transport worked. Idempotent — the `log` crate ignores a second
/// `set_logger`, and the `Once` guard avoids the racey double-set path.
#[cfg(any(feature = "reference-qemu", target_os = "nuttx"))]
fn install_stdout_logger() {
    struct StdoutLogger;
    impl log::Log for StdoutLogger {
        fn enabled(&self, _: &log::Metadata<'_>) -> bool {
            true
        }
        fn log(&self, record: &log::Record<'_>) {
            // The examples bake the full human line into the message
            // (`Publishing: '...'` / `I heard: [...]`), so emit it verbatim.
            // `[LEVEL]` prefix — parity with `nros_log`'s sink; see
            // nros-board-linux for why the tag is load-bearing.
            println!("[{}] {}", record.level(), record.args());
        }
        fn flush(&self) {}
    }
    static LOGGER: StdoutLogger = StdoutLogger;
    // phase-359 W7 — `std::sync::Once` became an atomic flag. `Once` is a
    // blocking primitive whose extra guarantee (a second caller WAITS for the
    // first to finish) buys nothing here: `log::set_logger` is itself atomic and
    // idempotent, so the guard's only job is to skip a redundant call. A
    // `compare_exchange` says exactly that and nothing more.
    static INIT: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
    if INIT
        .compare_exchange(
            false,
            true,
            core::sync::atomic::Ordering::AcqRel,
            core::sync::atomic::Ordering::Acquire,
        )
        .is_ok()
        && log::set_logger(&LOGGER).is_ok()
    {
        log::set_max_level(log::LevelFilter::Trace);
    }
}
